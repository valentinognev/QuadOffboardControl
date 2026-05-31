from __future__ import annotations

import csv
import os
import re
import stat
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, IO

import numpy as np
import torch
import torch.nn.functional as F
from torch.optim import Adam

from env_swarm_polygon import SwarmSearchPolygonEnv
from model import ActorCritic


@dataclass
class PPOConfig:
    total_env_steps: int = 3_500_000
    rollout_steps: int = 768
    gamma: float = 0.995
    gae_lambda: float = 0.95
    clip_range: float = 0.2
    learning_rate: float = 2.0e-4
    update_epochs: int = 6
    minibatch_size: int = 512
    entropy_coef: float = 0.01
    value_coef: float = 0.5
    frontier_aux_coef: float = 0.5
    guidance_aux_coef: float = 0.3
    marginal_aux_coef: float = 0.3
    territorial_aux_coef: float = 0.3
    max_grad_norm: float = 0.8
    eval_every: int = 25_000
    save_every: int = 25_000
    device: str = 'cpu'


class RolloutBuffer:
    def __init__(self):
        self.data = {k: [] for k in [
            'maps', 'self_state', 'neighbor_state', 'neighbor_mask',
            'global_maps', 'global_features',
            'actions', 'log_probs', 'rewards',
            'values', 'active_mask', 'done_mask', 'frontier_potential',
            'guidance_target', 'marginal_target', 'territorial_target',
        ]}

    def add(self, **kwargs):
        for k, v in kwargs.items():
            self.data[k].append(v)

    def clear(self):
        for k in self.data:
            self.data[k].clear()


class PPOTrainer:
    def __init__(self, env: SwarmSearchPolygonEnv, cfg: PPOConfig, out_dir: str):
        self.env = env
        self.cfg = cfg
        self.device = torch.device(cfg.device)

        self.model = ActorCritic(env.actor_obs_spec, env.critic_obs_spec).to(self.device)
        self.optimizer = Adam(self.model.parameters(), lr=cfg.learning_rate)

        self.global_step = 0
        self.episode_idx = 0
        self.best_eval_score = -1e18
        self.best_eval_step = 0
        self._last_save_step = 0
        self._last_eval_step = 0
        self.last_eval_score = None
        self.train_start_time = time.time()

        script_dir = self._to_posix_if_wsl(str(Path(__file__).resolve().parent))
        out_dir_str = self._to_posix_if_wsl(str(out_dir))
        if not os.path.isabs(out_dir_str):
            out_dir_str = os.path.join(script_dir, out_dir_str)
        self.out_dir = out_dir_str
        os.makedirs(self.out_dir, exist_ok=True)
        for sub in ['models', 'logs', 'videos', 'eval']:
            os.makedirs(os.path.join(self.out_dir, sub), exist_ok=True)

        self.train_csv = os.path.join(self.out_dir, 'logs', 'train_metrics.csv')
        self.loss_csv = os.path.join(self.out_dir, 'logs', 'ppo_losses.csv')
        self.eval_csv = os.path.join(self.out_dir, 'logs', 'eval_metrics.csv')
        self._init_csvs()

    @staticmethod
    def _to_posix_if_wsl(path: str) -> str:
        if os.name != 'nt':
            m = re.match(r'[\\/]{2,}wsl[.$\\/].*?[\\/]([^\\/]+)([\\/].*)', path)
            if m:
                return m.group(2).replace('\\', '/')
        return path

    _MAX_OPEN_RETRIES = 3
    _RETRY_DELAY = 0.5

    @staticmethod
    def _safe_open(filepath: str, mode: str = 'a') -> IO[str]:
        filepath = PPOTrainer._to_posix_if_wsl(filepath)
        filepath = os.path.realpath(filepath)
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        last_err: Exception | None = None
        for attempt in range(PPOTrainer._MAX_OPEN_RETRIES):
            try:
                return open(filepath, mode, newline='', encoding='utf-8')
            except PermissionError as exc:
                last_err = exc
                if os.path.exists(filepath):
                    try:
                        os.chmod(filepath, stat.S_IRUSR | stat.S_IWUSR | stat.S_IRGRP | stat.S_IROTH)
                    except OSError:
                        pass
                time.sleep(PPOTrainer._RETRY_DELAY * (attempt + 1))
        fallback = filepath + f'.{int(time.time())}.tmp'
        print(f'[warning] cannot write {filepath} ({last_err}); falling back to {fallback}', flush=True)
        return open(fallback, mode, newline='', encoding='utf-8')

    _TRAIN_FIELDS = [
        'env_steps', 'episode', 'reward_mean', 'coverage_mean',
        'ever_seen_fraction', 'percent_covered_at_least_once',
        'maintained_fraction', 'uncovered_fraction', 'stale_fraction',
        'active_agents', 'time_to_50_coverage', 'time_to_80_coverage',
        'time_to_95_coverage', 'overlap_ratio_mean', 'collision_penalty_mean',
        'min_inter_drone_dist', 'scan_efficiency_score',
    ]
    _LOSS_FIELDS = ['env_steps', 'policy_loss', 'value_loss', 'entropy', 'frontier_aux_loss', 'guidance_aux_loss', 'marginal_aux_loss', 'territorial_aux_loss', 'total_loss']
    _EVAL_FIELDS = ['env_steps', 'score', 'reward_mean', 'coverage_mean',
                    'ever_seen_fraction', 'maintained_fraction', 'scan_efficiency_score']

    def _init_csvs(self):
        with self._safe_open(self.train_csv, 'w') as f:
            csv.DictWriter(f, fieldnames=self._TRAIN_FIELDS).writeheader()
        with self._safe_open(self.loss_csv, 'w') as f:
            csv.DictWriter(f, fieldnames=self._LOSS_FIELDS).writeheader()
        with self._safe_open(self.eval_csv, 'w') as f:
            csv.DictWriter(f, fieldnames=self._EVAL_FIELDS).writeheader()

    def save_checkpoint(self, name: str):
        path = os.path.join(self.out_dir, 'models', name)
        torch.save({
            'model': self.model.state_dict(),
            'optimizer': self.optimizer.state_dict(),
            'global_step': self.global_step,
            'best_eval_score': self.best_eval_score,
            'best_eval_step': self.best_eval_step,
            'env_cfg': self.env.cfg.__dict__,
            'ppo_cfg': self.cfg.__dict__,
        }, path)

    def load_checkpoint(self, path: str):
        ckpt = torch.load(path, map_location=self.device)
        self.model.load_state_dict(ckpt['model'])
        self.optimizer.load_state_dict(ckpt['optimizer'])
        self.global_step = int(ckpt.get('global_step', 0))
        self.best_eval_score = float(ckpt.get('best_eval_score', -1e18))
        self.best_eval_step = int(ckpt.get('best_eval_step', 0))

    def _obs_to_tensors(self, obs: dict[str, np.ndarray]):
        return {k: torch.as_tensor(v, dtype=torch.float32, device=self.device) for k, v in obs.items()}

    def train(self):
        buf = RolloutBuffer()
        obs, info = self.env.reset(seed=0)
        n_agents = self.env.max_agents
        print(f'[train] starting: steps={self.cfg.total_env_steps} rollout={self.cfg.rollout_steps} '
              f'agents={n_agents} device={self.cfg.device}', flush=True)

        while self.global_step < self.cfg.total_env_steps:
            for _ in range(self.cfg.rollout_steps):
                obs_t = self._obs_to_tensors(obs)
                critic_obs = self.env.get_critic_observation()
                global_maps = torch.as_tensor(critic_obs['global_maps'], dtype=torch.float32, device=self.device).unsqueeze(0).expand(n_agents, -1, -1, -1)
                global_features = torch.as_tensor(critic_obs['global_features'], dtype=torch.float32, device=self.device).unsqueeze(0).expand(n_agents, -1)
                frontier_pot = torch.as_tensor(self.env.get_frontier_potential(), dtype=torch.float32, device=self.device)
                guidance_tgt = self.env.get_guidance_vector()  # (n_agents, 2)
                marginal_tgt = self.env.get_marginal_contribution()  # (n_agents,)
                territorial_tgt = self.env.get_territorial_fraction()  # (n_agents,)

                with torch.no_grad():
                    actions_t, log_probs_t = self.model.actor.act(
                        obs_t['maps'], obs_t['self_state'], obs_t['neighbor_state'], obs_t['neighbor_mask'],
                    )
                    values_t = self.model.critic(global_maps, global_features, obs_t['self_state'])

                actions_np = actions_t.cpu().numpy()
                next_obs, rewards, terminated, truncated, info = self.env.step(actions_np)
                done_flag = terminated or truncated

                buf.add(
                    maps=obs_t['maps'].cpu().numpy(),
                    self_state=obs_t['self_state'].cpu().numpy(),
                    neighbor_state=obs_t['neighbor_state'].cpu().numpy(),
                    neighbor_mask=obs_t['neighbor_mask'].cpu().numpy(),
                    global_maps=global_maps.cpu().numpy(),
                    global_features=global_features.cpu().numpy(),
                    actions=actions_np,
                    log_probs=log_probs_t.cpu().numpy(),
                    rewards=rewards.copy(),
                    values=values_t.cpu().numpy(),
                    active_mask=obs['active_mask'].copy(),
                    done_mask=np.full(n_agents, float(done_flag), dtype=np.float32),
                    frontier_potential=frontier_pot.cpu().numpy(),
                    guidance_target=guidance_tgt.copy(),
                    marginal_target=marginal_tgt.copy(),
                    territorial_target=territorial_tgt.copy(),
                )
                self.global_step += n_agents
                obs = next_obs

                if done_flag:
                    self.episode_idx += 1
                    self._log_train_row(self.episode_idx, info)
                    print(f"[train] step={self.global_step} ep={self.episode_idx} "
                          f"reward={info.get('reward_mean', 0):.3f} "
                          f"covered={info.get('percent_covered_at_least_once', 0):.1f}% "
                          f"maintained={info.get('maintained_fraction', 0):.3f} "
                          f"collision={info.get('collision_penalty_mean', 0):.3f} "
                          f"min_dist={info.get('min_inter_drone_dist', 0):.2f} "
                          f"score={info.get('scan_efficiency_score', 0):.1f}", flush=True)
                    obs, info = self.env.reset(seed=self.episode_idx)
                if self.global_step >= self.cfg.total_env_steps:
                    break

            obs_t = self._obs_to_tensors(obs)
            critic_obs = self.env.get_critic_observation()
            global_maps = torch.as_tensor(critic_obs['global_maps'], dtype=torch.float32, device=self.device).unsqueeze(0).expand(n_agents, -1, -1, -1)
            global_features = torch.as_tensor(critic_obs['global_features'], dtype=torch.float32, device=self.device).unsqueeze(0).expand(n_agents, -1)
            with torch.no_grad():
                last_values = self.model.critic(global_maps, global_features, obs_t['self_state']).cpu().numpy()

            loss_dict = self._ppo_update(buf, last_values)
            buf.clear()
            self._log_loss_row(loss_dict)
            elapsed = time.time() - self.train_start_time
            sps = self.global_step / max(1e-6, elapsed)
            # print(f"[rollout] step={self.global_step} episodes={self.episode_idx} "
            #       f"policy_loss={loss_dict['policy_loss']:.4f} value_loss={loss_dict['value_loss']:.4f} "
            #       f"entropy={loss_dict['entropy']:.4f} sps={sps:.0f}", flush=True)
            self.save_checkpoint('latest_checkpoint.pt')

            # if self.global_step - self._last_save_step >= self.cfg.save_every:
            #     self._last_save_step = self.global_step
            #     self.save_checkpoint(f'checkpoint_{self.global_step}.pt')

            if self.global_step - self._last_eval_step >= self.cfg.eval_every:
                self._last_eval_step = self.global_step
                eval_stats = self.evaluate(num_episodes=5)
                self._log_eval_row(eval_stats)
                score = float(eval_stats['score'])
                delta = None if self.last_eval_score is None else (score - self.last_eval_score)
                self.last_eval_score = score
                ds = 'n/a' if delta is None else f'{delta:+.3f}'
                elapsed = time.time() - self.train_start_time
                sps = self.global_step / max(1e-6, elapsed)
                print(f"[eval] step={self.global_step} score={score:.3f} (Δ {ds}) "
                      f"ever_seen={eval_stats['ever_seen_fraction']:.3f} "
                      f"maintained={eval_stats['maintained_fraction']:.3f} "
                      f"best={self.best_eval_score:.3f} sps={sps:.0f}", flush=True)
                if score > self.best_eval_score:
                    self.best_eval_score = score
                    self.best_eval_step = self.global_step
                    self.save_checkpoint('best_model.pt')
                    self.save_checkpoint('best_so_far.pt')
                    print(f'[eval] ★ new best at step={self.global_step} score={score:.3f}', flush=True)

        self.save_checkpoint('last_model.pt')
        print(f'[train] done. final_step={self.global_step} best_score={self.best_eval_score:.3f} '
              f'at step={self.best_eval_step}', flush=True)

    def _masked_mean(self, x: torch.Tensor, mask: torch.Tensor) -> torch.Tensor:
        denom = mask.sum().clamp(min=1.0)
        return (x * mask).sum() / denom

    def _ppo_update(self, buf: RolloutBuffer, last_value: np.ndarray) -> Dict[str, float]:
        d = buf.data
        rewards = np.asarray(d['rewards'], dtype=np.float32)
        dones = np.asarray(d['done_mask'], dtype=np.float32)
        values = np.asarray(d['values'], dtype=np.float32)
        active = np.asarray(d['active_mask'], dtype=np.float32)
        t_steps, n_agents = rewards.shape

        advantages = np.zeros_like(rewards)
        last_gae = np.zeros(n_agents, dtype=np.float32)
        next_values = last_value.astype(np.float32)
        for t in reversed(range(t_steps)):
            nonterminal = 1.0 - dones[t]
            delta = rewards[t] + self.cfg.gamma * next_values * nonterminal - values[t]
            last_gae = delta + self.cfg.gamma * self.cfg.gae_lambda * nonterminal * last_gae
            advantages[t] = last_gae
            next_values = values[t]
        returns = advantages + values

        maps = torch.as_tensor(np.concatenate(d['maps']), dtype=torch.float32, device=self.device)
        self_states = torch.as_tensor(np.concatenate(d['self_state']), dtype=torch.float32, device=self.device)
        nbr_states = torch.as_tensor(np.concatenate(d['neighbor_state']), dtype=torch.float32, device=self.device)
        nbr_masks = torch.as_tensor(np.concatenate(d['neighbor_mask']), dtype=torch.float32, device=self.device)
        global_maps = torch.as_tensor(np.concatenate(d['global_maps']), dtype=torch.float32, device=self.device)
        global_features = torch.as_tensor(np.concatenate(d['global_features']), dtype=torch.float32, device=self.device)
        actions = torch.as_tensor(np.concatenate(d['actions']), dtype=torch.float32, device=self.device)
        old_log_probs = torch.as_tensor(np.concatenate(d['log_probs']), dtype=torch.float32, device=self.device)
        frontier_targets = torch.as_tensor(np.concatenate(d['frontier_potential']), dtype=torch.float32, device=self.device)
        guidance_targets = torch.as_tensor(np.concatenate(d['guidance_target']), dtype=torch.float32, device=self.device)
        marginal_targets = torch.as_tensor(np.concatenate(d['marginal_target']), dtype=torch.float32, device=self.device)
        territorial_targets = torch.as_tensor(np.concatenate(d['territorial_target']), dtype=torch.float32, device=self.device)
        active_flat = torch.as_tensor(active.reshape(-1), dtype=torch.float32, device=self.device)
        adv_t = torch.as_tensor(advantages.reshape(-1), dtype=torch.float32, device=self.device)
        ret_t = torch.as_tensor(returns.reshape(-1), dtype=torch.float32, device=self.device)

        active_adv = adv_t[active_flat > 0.5]
        if active_adv.numel() > 1:
            mean = active_adv.mean()
            std = active_adv.std().clamp(min=1e-8)
            adv_t = torch.where(active_flat > 0.5, (adv_t - mean) / std, torch.zeros_like(adv_t))
        else:
            adv_t = torch.zeros_like(adv_t)

        total_policy = total_value = total_entropy = total_frontier = 0.0
        total_guidance = total_marginal = total_territorial = 0.0
        count = 0
        idxs = np.arange(actions.shape[0])
        for _ in range(self.cfg.update_epochs):
            np.random.shuffle(idxs)
            for start in range(0, len(idxs), self.cfg.minibatch_size):
                mb = idxs[start:start + self.cfg.minibatch_size]
                mask_mb = active_flat[mb]
                new_lp, entropy = self.model.actor.evaluate_actions(
                    maps[mb], self_states[mb], nbr_states[mb], nbr_masks[mb], actions[mb],
                )
                v_pred, frontier_pred, guidance_pred, _corner_pred, marginal_pred, territorial_pred = self.model.critic.forward_with_frontier(
                    global_maps[mb], global_features[mb], self_states[mb],
                )

                ratio = torch.exp(new_lp - old_log_probs[mb])
                s1 = ratio * adv_t[mb]
                s2 = torch.clamp(ratio, 1.0 - self.cfg.clip_range, 1.0 + self.cfg.clip_range) * adv_t[mb]
                policy_loss = -self._masked_mean(torch.min(s1, s2), mask_mb)
                value_loss = self._masked_mean((v_pred - ret_t[mb]).pow(2), mask_mb)
                entropy_mean = self._masked_mean(entropy, mask_mb)
                frontier_loss = self._masked_mean((frontier_pred - frontier_targets[mb]).pow(2), mask_mb)
                guidance_loss = self._masked_mean((guidance_pred - guidance_targets[mb]).pow(2).sum(dim=-1), mask_mb)
                marginal_loss = self._masked_mean((marginal_pred - marginal_targets[mb]).pow(2), mask_mb)
                territorial_loss = self._masked_mean((territorial_pred - territorial_targets[mb]).pow(2), mask_mb)

                total_loss = (
                    policy_loss
                    + self.cfg.value_coef * value_loss
                    - self.cfg.entropy_coef * entropy_mean
                    + self.cfg.frontier_aux_coef * frontier_loss
                    + self.cfg.guidance_aux_coef * guidance_loss
                    + self.cfg.marginal_aux_coef * marginal_loss
                    + self.cfg.territorial_aux_coef * territorial_loss
                )

                self.optimizer.zero_grad()
                total_loss.backward()
                torch.nn.utils.clip_grad_norm_(self.model.parameters(), self.cfg.max_grad_norm)
                self.optimizer.step()

                total_policy += float(policy_loss.item())
                total_value += float(value_loss.item())
                total_entropy += float(entropy_mean.item())
                total_frontier += float(frontier_loss.item())
                total_guidance += float(guidance_loss.item())
                total_marginal += float(marginal_loss.item())
                total_territorial += float(territorial_loss.item())
                count += 1

        return {
            'policy_loss': total_policy / max(1, count),
            'value_loss': total_value / max(1, count),
            'entropy': total_entropy / max(1, count),
            'frontier_aux_loss': total_frontier / max(1, count),
            'guidance_aux_loss': total_guidance / max(1, count),
            'marginal_aux_loss': total_marginal / max(1, count),
            'territorial_aux_loss': total_territorial / max(1, count),
            'total_loss': (total_policy + total_value + total_frontier + total_guidance + total_marginal + total_territorial) / max(1, count),
        }

    def evaluate(self, num_episodes: int = 5) -> Dict[str, float]:
        """Evaluate the current policy over multiple episodes.

        The evaluation score uses a GATED / LEXICOGRAPHIC structure:
          Gate 1: SAFETY — any collision (drone-drone or wall) → score = 0
          Gate 2: COVERAGE COMPLETION — must reach 100% ever_seen; near-100% is NOT success
          Gate 3: HOLE CLEANUP QUALITY — penalize leftover uncovered cells
          Gate 4: EFFICIENCY & BALANCE — speed, fairness, maintenance quality

        This score is used ONLY for model selection/checkpointing.
        It never enters the reward or gradient computation.
        """
        ep_scores = []
        ep_evers = []
        ep_mains = []
        ep_rewards = []

        for ep in range(num_episodes):
            obs, info = self.env.reset(seed=10_000 + ep)
            total_reward = 0.0

            # Per-episode tracking
            step_count = 0
            safety_violated = False
            collision_count = 0
            wall_collision_count = 0

            for _ in range(self.env.max_steps):
                obs_t = self._obs_to_tensors(obs)
                with torch.no_grad():
                    actions_t, _ = self.model.actor.act(
                        obs_t['maps'], obs_t['self_state'], obs_t['neighbor_state'], obs_t['neighbor_mask'],
                        deterministic=True,
                    )
                obs, r, terminated, truncated, info = self.env.step(actions_t.cpu().numpy())
                active = max(1.0, float(np.sum(obs['active_mask'])))
                total_reward += float(np.sum(r) / active)
                step_count += 1
                if terminated or truncated:
                    break

            # ── Collect end-of-episode metrics from env ──
            ever_seen = info.get('ever_seen_fraction', 0.0)
            maintained = info.get('maintained_fraction', 0.0)
            uncovered_cells = info.get('uncovered_cells', 0)
            total_active_cells = info.get('total_active_cells', 1)
            safety_violated = info.get('safety_violated', False)
            collision_count = info.get('episode_collision_count', 0)
            wall_collision_count = info.get('episode_wall_collision_count', 0)

            # ══════════════════════════════════════════════════════════
            #  GATE 1: SAFETY — hard zero if ANY collision occurred
            # ══════════════════════════════════════════════════════════
            if safety_violated or collision_count > 0 or wall_collision_count > 0:
                ep_scores.append(0.0)
                ep_evers.append(ever_seen)
                ep_mains.append(maintained)
                ep_rewards.append(total_reward)
                continue

            # ══════════════════════════════════════════════════════════
            #  GATE 2: COVERAGE COMPLETION — 100% is the threshold
            # ══════════════════════════════════════════════════════════
            # Models that reach 100% are in a fundamentally different tier.
            # Near-100% (99.5%) is NOT success — it means holes remain.
            reached_100 = (ever_seen >= 1.0 - 1e-6)

            if not reached_100:
                # Score range: [1, 30] — always below any 100%-coverage episode
                # Scaled by how close to 100% (rewards partial progress but caps low)
                coverage_progress = min(ever_seen, 0.999)
                # Non-linear: 99% → ~25, 95% → ~15, 80% → ~5
                gate2_score = 1.0 + 29.0 * (coverage_progress ** 4)
                ep_scores.append(gate2_score)
                ep_evers.append(ever_seen)
                ep_mains.append(maintained)
                ep_rewards.append(total_reward)
                continue

            # ══════════════════════════════════════════════════════════
            #  GATE 3: HOLE CLEANUP QUALITY (for 100% episodes)
            # ══════════════════════════════════════════════════════════
            # How quickly did we reach 100%? Late completion = holes were hard to find.
            ep_len = self.env.cfg.episode_seconds
            t95 = info.get('time_to_95_coverage', ep_len)
            t95 = t95 if t95 >= 0 else ep_len

            # Time between 95% and 100%: measures hole-hunting efficiency
            # Total episode time used
            time_used_frac = step_count / max(1, self.env.max_steps)
            # Reward finishing with time to spare (0 = used entire episode, 1 = instant)
            completion_speed = max(0.0, 1.0 - time_used_frac)

            # Speed to 95%: measures bulk coverage efficiency
            speed_95 = max(0.0, 1.0 - (t95 / ep_len)) if t95 >= 0 else 0.0

            # Hole cleanup score: how much episode was spent on the last 5%
            # If 95% reached early and 100% reached quickly after → high score
            cleanup_phase_frac = max(0.0, time_used_frac - (t95 / ep_len)) if t95 >= 0 else time_used_frac
            # Ideal: cleanup takes <10% of episode. Penalty grows if it takes longer.
            cleanup_efficiency = max(0.0, 1.0 - cleanup_phase_frac / 0.30)  # 0 if >30% spent on last 5%

            hole_quality = 0.4 * completion_speed + 0.3 * speed_95 + 0.3 * cleanup_efficiency

            # ══════════════════════════════════════════════════════════
            #  GATE 4: EFFICIENCY & BALANCE
            # ══════════════════════════════════════════════════════════

            # 4a) Contribution fairness (Gini coefficient of per-drone discoveries)
            drone_disc = self.env._drone_discoveries.copy()
            active_mask = self.env.active_mask
            active_disc = drone_disc[active_mask > 0.5].astype(np.float64)
            n_active = len(active_disc)

            if n_active > 1 and np.sum(active_disc) > 0:
                sorted_d = np.sort(active_disc)
                index = np.arange(1, n_active + 1, dtype=np.float64)
                gini = float((2.0 * np.sum(index * sorted_d) - (n_active + 1) * np.sum(sorted_d)) /
                             (n_active * np.sum(sorted_d)))
                # gini: 0 = perfect equality, 1 = one drone did everything
                fairness_score = max(0.0, 1.0 - gini * 2.0)  # 0 if gini >= 0.5
            else:
                fairness_score = 1.0 if n_active <= 1 else 0.0

            # 4b) Maintenance quality (need-reduction-based, only after 100%)
            # Uses the smart maintenance metric: low mean need + uniform + fair
            maint_q = info.get('maint_quality', {})
            maintenance_score = float(maint_q.get('quality_score', maintained))

            # 4c) Overlap efficiency: low overlap = good spatial division
            overlap_mean = info.get('overlap_ratio_mean', 0.0)
            overlap_score = max(0.0, 1.0 - overlap_mean * 3.0)  # 0 if overlap > 0.33

            # ── Combine Gate 3 + Gate 4 into final score ──
            # Score range for 100%-safe episodes: [30, 100]
            # Gate 3 (hole quality): 50% weight — most important differentiator
            # Gate 4a (fairness): 20% weight
            # Gate 4b (maintenance): 15% weight
            # Gate 4c (overlap/spatial efficiency): 15% weight
            quality = (0.50 * hole_quality
                       + 0.20 * fairness_score
                       + 0.15 * maintenance_score
                       + 0.15 * overlap_score)

            # Map to [30, 100] range (30 = worst 100%-safe, 100 = perfect)
            episode_score = 30.0 + 70.0 * quality

            ep_scores.append(episode_score)
            ep_evers.append(ever_seen)
            ep_mains.append(maintained)
            ep_rewards.append(total_reward)

        return {
            'env_steps': self.global_step,
            'score': float(np.mean(ep_scores)),
            'reward_mean': float(np.mean(ep_rewards)),
            'coverage_mean': float(np.mean(ep_evers)),
            'ever_seen_fraction': float(np.mean(ep_evers)),
            'maintained_fraction': float(np.mean(ep_mains)),
            'scan_efficiency_score': float(np.mean(ep_scores)),
        }

    def _log_train_row(self, episode: int, info: Dict[str, float]):
        with self._safe_open(self.train_csv, 'a') as f:
            writer = csv.DictWriter(f, fieldnames=self._TRAIN_FIELDS)
            row = {'env_steps': self.global_step, 'episode': episode}
            for k in self._TRAIN_FIELDS:
                if k not in row:
                    row[k] = info.get(k, 0.0)
            writer.writerow(row)

    def _log_loss_row(self, loss_dict: Dict[str, float]):
        with self._safe_open(self.loss_csv, 'a') as f:
            csv.DictWriter(f, fieldnames=self._LOSS_FIELDS).writerow({'env_steps': self.global_step, **loss_dict})

    def _log_eval_row(self, eval_stats: Dict[str, float]):
        with self._safe_open(self.eval_csv, 'a') as f:
            csv.DictWriter(f, fieldnames=self._EVAL_FIELDS).writerow(eval_stats)
