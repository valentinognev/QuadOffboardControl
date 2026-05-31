"""PPO Trainer with vectorized (parallel) environments for faster training."""
from __future__ import annotations

import csv
import os
import re
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, IO

import numpy as np
import torch
from torch.optim import Adam

from env_swarm_polygon import EnvConfig, SwarmSearchPolygonEnv
from model import ActorCritic
from vec_env import VecEnv


@dataclass
class PPOConfig:
    total_env_steps: int = 3_500_000
    num_workers: int = 1
    num_envs: int = 12 #environments per worker
    rollout_steps: int = 192  # per env (total = num_workers * num_envs * rollout_steps * agents)
    gamma: float = 0.995
    gae_lambda: float = 0.95
    clip_range: float = 0.2
    learning_rate: float = 2.5e-4
    update_epochs: int = 6
    minibatch_size: int = 1024  # larger batch since more data
    entropy_coef: float = 0.01
    value_coef: float = 0.5
    frontier_aux_coef: float = 0.5
    marginal_aux_coef: float = 0.5
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
            'guidance_vector', 'marginal_contribution', 'territorial_fraction',
        ]}

    def add(self, **kwargs):
        for k, v in kwargs.items():
            self.data[k].append(v)

    def clear(self):
        for k in self.data:
            self.data[k].clear()


class PPOTrainerVec:
    """Vectorized PPO trainer — runs num_envs environments in parallel."""

    def __init__(self, env_cfg: EnvConfig, cfg: PPOConfig, out_dir: str):
        self.cfg = cfg
        self.device = torch.device(cfg.device)
        self.env_cfg = env_cfg

        # Vectorized envs for training
        self.vec_env = VecEnv(env_cfg, num_workers=cfg.num_workers, num_envs_per_worker=cfg.num_envs)
        self.n_agents = self.vec_env.max_agents

        # Single env for evaluation
        self.eval_env = SwarmSearchPolygonEnv(env_cfg)

        self.model = ActorCritic(self.vec_env.actor_obs_spec, self.vec_env.critic_obs_spec).to(self.device)
        # Separate optimizers so critic-only auxiliary losses don't update actor params
        self.optimizer_actor = Adam(self.model.actor.parameters(), lr=cfg.learning_rate)
        self.optimizer_critic = Adam(self.model.critic.parameters(), lr=cfg.learning_rate)

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

    @staticmethod
    def _safe_open(filepath: str, mode: str = 'a') -> IO[str]:
        filepath = PPOTrainerVec._to_posix_if_wsl(filepath)
        filepath = os.path.realpath(filepath)
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        try:
            return open(filepath, mode, newline='', encoding='utf-8')
        except PermissionError:
            fallback = filepath + f'.{int(time.time())}.tmp'
            return open(fallback, mode, newline='', encoding='utf-8')

    _TRAIN_FIELDS = [
        'env_steps', 'episode', 'reward_mean', 'coverage_mean',
        'ever_seen_fraction', 'percent_covered_at_least_once',
        'maintained_fraction', 'uncovered_fraction', 'stale_fraction',
        'active_agents', 'time_to_50_coverage', 'time_to_80_coverage',
        'time_to_95_coverage', 'overlap_ratio_mean', 'collision_penalty_mean',
        'min_inter_drone_dist', 'scan_efficiency_score',
    ]
    _LOSS_FIELDS = ['env_steps', 'policy_loss', 'value_loss', 'entropy', 'frontier_aux_loss', 'corner_aux_loss', 'marginal_aux_loss', 'territorial_aux_loss', 'total_loss']
    _EVAL_FIELDS = ['env_steps', 'score', 'reward_mean', 'coverage_mean',
                    'ever_seen_fraction', 'ever_seen_min', 'maintained_fraction', 'scan_efficiency_score']

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
            'optimizer_actor': self.optimizer_actor.state_dict(),
            'optimizer_critic': self.optimizer_critic.state_dict(),
            'global_step': self.global_step,
            'best_eval_score': self.best_eval_score,
            'best_eval_step': self.best_eval_step,
        }, path)

    def load_checkpoint(self, path: str):
        ckpt = torch.load(path, map_location=self.device)
        sd = ckpt.get('model', ckpt)

        # Try exact load first
        try:
            self.model.load_state_dict(sd)
            print(f'  [checkpoint] Exact load successful.')
        except RuntimeError:
            # Try strict=False (handles missing keys but not shape mismatches)
            try:
                res = self.model.load_state_dict(sd, strict=False)
                print(f'  [checkpoint] Partial load (strict=False): '
                      f'missing={len(res.missing_keys)}, unexpected={len(res.unexpected_keys)}')
            except RuntimeError:
                # Manual copy: only load tensors that match by name AND shape
                own_state = self.model.state_dict()
                copied, skipped = 0, 0
                for k, v in sd.items():
                    if k in own_state and own_state[k].shape == v.shape:
                        own_state[k] = v
                        copied += 1
                    else:
                        skipped += 1
                self.model.load_state_dict(own_state)
                print(f'  [checkpoint] Manual load: copied={copied}, skipped={skipped} (shape mismatch or missing)')

        # Restore optimizer states (best-effort)
        if 'optimizer_actor' in ckpt and 'optimizer_critic' in ckpt:
            try:
                self.optimizer_actor.load_state_dict(ckpt['optimizer_actor'])
                self.optimizer_critic.load_state_dict(ckpt['optimizer_critic'])
            except Exception:
                print(f'  [checkpoint] Optimizer state load failed — using fresh optimizers.')

        self.global_step = int(ckpt.get('global_step', 0))
        self.best_eval_score = float(ckpt.get('best_eval_score', -1e18))
        self.best_eval_step = int(ckpt.get('best_eval_step', 0))

    def train(self):
        buf = RolloutBuffer()
        W = self.cfg.num_workers
        E = self.cfg.num_envs
        N = W * E
        A = self.n_agents

        obs, _ = self.vec_env.reset_all(base_seed=0)
        # obs is dict of (num_envs, agents, ...) arrays

        print(f'[train-vec] starting: total_steps={self.cfg.total_env_steps} '
              f'num_envs={N} rollout={self.cfg.rollout_steps} agents={A} '
              f'device={self.cfg.device}', flush=True)
        print(f'[train-vec] effective batch per rollout: {N * self.cfg.rollout_steps * A} transitions', flush=True)

        while self.global_step < self.cfg.total_env_steps:
            # Collect rollout
            for _ in range(self.cfg.rollout_steps):
                # obs: dict with shape (N, A, ...)
                obs_t = {k: torch.as_tensor(v.reshape(N * A, *v.shape[2:]), dtype=torch.float32, device=self.device)
                         for k, v in obs.items()}

                critic_obs_list = self.vec_env.get_critic_observations()
                global_maps = np.stack([c['global_maps'] for c in critic_obs_list])  # (N, C, H, W)
                global_features = np.stack([c['global_features'] for c in critic_obs_list])  # (N, F)
                # Expand to per-agent
                global_maps_exp = np.repeat(global_maps, A, axis=0)  # (N*A, C, H, W)
                global_features_exp = np.repeat(global_features, A, axis=0)  # (N*A, F)
                global_maps_t = torch.as_tensor(global_maps_exp, dtype=torch.float32, device=self.device)
                global_features_t = torch.as_tensor(global_features_exp, dtype=torch.float32, device=self.device)

                frontier_pot = self.vec_env.get_frontier_potentials()  # (N, A)
                guidance_vec = self.vec_env.get_guidance_vectors()  # (N, A, 2) — combined direction target
                marginal_cont = self.vec_env.get_marginal_contributions()  # (N, A)
                territorial_frac = self.vec_env.get_territorial_fractions()  # (N, A)

                with torch.no_grad():
                    actions_t, log_probs_t = self.model.actor.act(
                        obs_t['maps'], obs_t['self_state'], obs_t['neighbor_state'], obs_t['neighbor_mask'],
                    )
                    values_t = self.model.critic(global_maps_t, global_features_t, obs_t['self_state'])

                actions_np = actions_t.cpu().numpy().reshape(N, A, -1)
                next_obs, rewards, dones, infos = self.vec_env.step(actions_np)

                # Store flat (N*A) arrays
                buf.add(
                    maps=obs_t['maps'].cpu().numpy(),
                    self_state=obs_t['self_state'].cpu().numpy(),
                    neighbor_state=obs_t['neighbor_state'].cpu().numpy(),
                    neighbor_mask=obs_t['neighbor_mask'].cpu().numpy(),
                    global_maps=global_maps_exp,
                    global_features=global_features_exp,
                    actions=actions_t.cpu().numpy(),
                    log_probs=log_probs_t.cpu().numpy(),
                    rewards=rewards.reshape(N * A),
                    values=values_t.cpu().numpy(),
                    active_mask=obs['active_mask'].reshape(N * A),
                    done_mask=np.repeat(dones.astype(np.float32), A),
                    frontier_potential=frontier_pot.reshape(N * A),
                    guidance_vector=guidance_vec.reshape(N * A, 2),
                    marginal_contribution=marginal_cont.reshape(N * A),
                    territorial_fraction=territorial_frac.reshape(N * A),
                )

                self.global_step += N * A
                obs = next_obs

                # Log terminal episodes
                for i, info in enumerate(infos):
                    if info.get('_terminal', False):
                        self.episode_idx += 1
                        # Log train metrics to CSV
                        self._log_train_row(info)
                        # Save model on 100% coverage
                        es = info.get('ever_seen_fraction', 0.0)
                        if es >= 0.999:
                            self.save_checkpoint('model_100coverage.pt')
                            print(f'[train-vec] ★ 100% coverage at step={self.global_step}!', flush=True)
                        if self.episode_idx % 5 == 0:
                            elapsed = time.time() - self.train_start_time
                            sps = self.global_step / max(1e-6, elapsed)
                            print(f"[train-vec] step={self.global_step} ep={self.episode_idx} "
                                  f"reward={info.get('reward_mean', 0):.3f} "
                                  f"covered={info.get('percent_covered_at_least_once', 0):.1f}% "
                                  f"maintained={info.get('maintained_fraction', 0):.3f} "
                                  f"sps={sps:.0f}", flush=True)

                if self.global_step >= self.cfg.total_env_steps:
                    break

            # Compute last values for GAE
            obs_t = {k: torch.as_tensor(v.reshape(N * A, *v.shape[2:]), dtype=torch.float32, device=self.device)
                     for k, v in obs.items()}
            critic_obs_list = self.vec_env.get_critic_observations()
            global_maps = np.stack([c['global_maps'] for c in critic_obs_list])
            global_features = np.stack([c['global_features'] for c in critic_obs_list])
            global_maps_exp = np.repeat(global_maps, A, axis=0)
            global_features_exp = np.repeat(global_features, A, axis=0)
            with torch.no_grad():
                last_values = self.model.critic(
                    torch.as_tensor(global_maps_exp, dtype=torch.float32, device=self.device),
                    torch.as_tensor(global_features_exp, dtype=torch.float32, device=self.device),
                    obs_t['self_state'],
                ).cpu().numpy()

            loss_dict = self._ppo_update(buf, last_values)
            buf.clear()

            # Log losses to CSV
            loss_row = {'env_steps': self.global_step, **loss_dict}
            with self._safe_open(self.loss_csv, 'a') as f:
                csv.DictWriter(f, fieldnames=self._LOSS_FIELDS).writerow(loss_row)

            elapsed = time.time() - self.train_start_time
            sps = self.global_step / max(1e-6, elapsed)

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
                print(f"[eval] step={self.global_step} score={score:.3f} (Δ {ds}) "
                      f"ever_seen={eval_stats['ever_seen_fraction']:.3f} "
                      f"maintained={eval_stats['maintained_fraction']:.3f} "
                      f"best={self.best_eval_score:.3f} sps={sps:.0f}", flush=True)
                if score > self.best_eval_score:
                    self.best_eval_score = score
                    self.best_eval_step = self.global_step
                    self.save_checkpoint('best_model.pt')
                    print(f'[eval] ★ new best at step={self.global_step} score={score:.3f}', flush=True)
                # Save 100% coverage model only if ALL eval episodes hit ≥99%
                # (min, not mean — ensures generalization across polygon shapes)
                if eval_stats.get('ever_seen_min', 0.0) >= 0.99:
                    self.save_checkpoint('model_100coverage.pt')
                    print(f'[eval] 🏆 100% coverage model saved at step={self.global_step} '
                          f'(min={eval_stats["ever_seen_min"]:.3f})', flush=True)

        self.vec_env.close()
        self.save_checkpoint('last_model.pt')
        print(f'[train-vec] done. final_step={self.global_step} best={self.best_eval_score:.3f}', flush=True)

    def _masked_mean(self, x: torch.Tensor, mask: torch.Tensor) -> torch.Tensor:
        return (x * mask).sum() / mask.sum().clamp(min=1.0)

    def _ppo_update(self, buf: RolloutBuffer, last_value: np.ndarray) -> Dict[str, float]:
        d = buf.data
        rewards = np.asarray(d['rewards'], dtype=np.float32)  # (T, N*A)
        dones = np.asarray(d['done_mask'], dtype=np.float32)
        values = np.asarray(d['values'], dtype=np.float32)
        active = np.asarray(d['active_mask'], dtype=np.float32)
        t_steps, flat_agents = rewards.shape

        advantages = np.zeros_like(rewards)
        last_gae = np.zeros(flat_agents, dtype=np.float32)
        next_values = last_value.astype(np.float32)
        for t in reversed(range(t_steps)):
            nonterminal = 1.0 - dones[t]
            delta = rewards[t] + self.cfg.gamma * next_values * nonterminal - values[t]
            last_gae = delta + self.cfg.gamma * self.cfg.gae_lambda * nonterminal * last_gae
            advantages[t] = last_gae
            next_values = values[t]
        returns = advantages + values

        # Flatten everything
        maps = torch.as_tensor(np.concatenate(d['maps']), dtype=torch.float32, device=self.device)
        self_states = torch.as_tensor(np.concatenate(d['self_state']), dtype=torch.float32, device=self.device)
        nbr_states = torch.as_tensor(np.concatenate(d['neighbor_state']), dtype=torch.float32, device=self.device)
        nbr_masks = torch.as_tensor(np.concatenate(d['neighbor_mask']), dtype=torch.float32, device=self.device)
        global_maps_t = torch.as_tensor(np.concatenate(d['global_maps']), dtype=torch.float32, device=self.device)
        global_features_t = torch.as_tensor(np.concatenate(d['global_features']), dtype=torch.float32, device=self.device)
        actions = torch.as_tensor(np.concatenate(d['actions']), dtype=torch.float32, device=self.device)
        old_log_probs = torch.as_tensor(np.concatenate(d['log_probs']), dtype=torch.float32, device=self.device)
        frontier_targets = torch.as_tensor(np.concatenate(d['frontier_potential']), dtype=torch.float32, device=self.device)
        guidance_targets = torch.as_tensor(np.concatenate(d['guidance_vector']), dtype=torch.float32, device=self.device)
        marginal_targets = torch.as_tensor(np.concatenate(d['marginal_contribution']), dtype=torch.float32, device=self.device)
        territorial_targets = torch.as_tensor(np.concatenate(d['territorial_fraction']), dtype=torch.float32, device=self.device)
        active_flat = torch.as_tensor(active.reshape(-1), dtype=torch.float32, device=self.device)
        adv_t = torch.as_tensor(advantages.reshape(-1), dtype=torch.float32, device=self.device)
        ret_t = torch.as_tensor(returns.reshape(-1), dtype=torch.float32, device=self.device)

        active_adv = adv_t[active_flat > 0.5]
        if active_adv.numel() > 1:
            mean, std = active_adv.mean(), active_adv.std().clamp(min=1e-8)
            adv_t = torch.where(active_flat > 0.5, (adv_t - mean) / std, torch.zeros_like(adv_t))

        total_policy = total_value = total_entropy = total_frontier = total_corner = 0.0
        total_marginal = total_territorial = 0.0
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
                v_pred, frontier_pred, guidance_pred, corner_pred, marginal_pred, territorial_pred = self.model.critic.forward_with_frontier(
                    global_maps_t[mb], global_features_t[mb], self_states[mb],
                )
                ratio = torch.exp(new_lp - old_log_probs[mb])
                s1 = ratio * adv_t[mb]
                s2 = torch.clamp(ratio, 1.0 - self.cfg.clip_range, 1.0 + self.cfg.clip_range) * adv_t[mb]
                policy_loss = -self._masked_mean(torch.min(s1, s2), mask_mb)
                value_loss = self._masked_mean((v_pred - ret_t[mb]).pow(2), mask_mb)
                entropy_mean = self._masked_mean(entropy, mask_mb)
                frontier_loss = self._masked_mean((frontier_pred - frontier_targets[mb]).pow(2), mask_mb)
                guidance_loss = self._masked_mean(
                    (guidance_pred - guidance_targets[mb]).pow(2).sum(dim=-1), mask_mb)

                # ---- Actor update (actor params only) ----
                policy_loss = -self._masked_mean(torch.min(s1, s2), mask_mb)
                entropy_mean = self._masked_mean(entropy, mask_mb)
                actor_loss = policy_loss - self.cfg.entropy_coef * entropy_mean
                self.optimizer_actor.zero_grad()
                actor_loss.backward()
                torch.nn.utils.clip_grad_norm_(self.model.actor.parameters(), self.cfg.max_grad_norm)
                self.optimizer_actor.step()

                # ---- Critic update (critic params only) ----
                value_loss = self._masked_mean((v_pred - ret_t[mb]).pow(2), mask_mb)
                frontier_loss = self._masked_mean((frontier_pred - frontier_targets[mb]).pow(2), mask_mb)
                marginal_loss = self._masked_mean((marginal_pred - marginal_targets[mb]).pow(2), mask_mb)
                territorial_loss = self._masked_mean((territorial_pred - territorial_targets[mb]).pow(2), mask_mb)
                corner_loss = torch.tensor(0.0, device=self.device)
                num_corner_feats = self.vec_env.critic_obs_spec.get('num_corners', 0) * self.vec_env.critic_obs_spec.get('corner_feat_per', 0)
                if corner_pred is not None and num_corner_feats > 0:
                    corner_targets = global_features_t[mb][:, -num_corner_feats:]
                    corner_loss = self._masked_mean((corner_pred - corner_targets).pow(2).mean(dim=-1), mask_mb)

                critic_loss = (
                    self.cfg.value_coef * value_loss
                    + self.cfg.frontier_aux_coef * (frontier_loss + guidance_loss)
                    + self.cfg.marginal_aux_coef * marginal_loss
                    + self.cfg.territorial_aux_coef * territorial_loss
                    + getattr(self.cfg, 'corner_aux_coef', 1.0) * corner_loss
                )
                self.optimizer_critic.zero_grad()
                critic_loss.backward()
                torch.nn.utils.clip_grad_norm_(self.model.critic.parameters(), self.cfg.max_grad_norm)
                self.optimizer_critic.step()

                total_policy += float(policy_loss.item())
                total_value += float(value_loss.item())
                total_entropy += float(entropy_mean.item())
                total_frontier += float(frontier_loss.item())
                total_marginal += float(marginal_loss.item())
                total_territorial += float(territorial_loss.item())
                # accumulate corner loss scalar
                try:
                    total_corner += float(corner_loss.item())
                except Exception:
                    total_corner += float(corner_loss)
                count += 1

        return {
            'policy_loss': total_policy / max(1, count),
            'value_loss': total_value / max(1, count),
            'entropy': total_entropy / max(1, count),
            'frontier_aux_loss': total_frontier / max(1, count),
            'corner_aux_loss': total_corner / max(1, count),
            'marginal_aux_loss': total_marginal / max(1, count),
            'territorial_aux_loss': total_territorial / max(1, count),
            'total_loss': (total_policy + total_value + total_frontier + total_corner + total_marginal + total_territorial) / max(1, count),
        }

    def evaluate(self, num_episodes: int = 5) -> Dict[str, float]:
        rewards, evers, mains, scores = [], [], [], []
        for ep in range(num_episodes):
            obs, _ = self.eval_env.reset(seed=10_000 + ep)
            total = 0.0
            maintained_sum = 0.0
            n_steps = 0
            for _ in range(self.eval_env.max_steps):
                obs_t = {k: torch.as_tensor(v, dtype=torch.float32, device=self.device) for k, v in obs.items()}
                with torch.no_grad():
                    actions_t, _ = self.model.actor.act(
                        obs_t['maps'], obs_t['self_state'], obs_t['neighbor_state'], obs_t['neighbor_mask'],
                        deterministic=True,
                    )
                obs, r, terminated, truncated, info = self.eval_env.step(actions_t.cpu().numpy())
                total += float(np.sum(r) / max(1.0, float(np.sum(obs['active_mask']))))
                maintained_sum += info.get('maintained_fraction', 0.0)
                n_steps += 1
                if terminated or truncated:
                    break
            rewards.append(total)
            evers.append(info.get('ever_seen_fraction', 0.0))
            mains.append(maintained_sum / max(1, n_steps))  # average maintained over episode
            scores.append(info.get('scan_efficiency_score', 0.0))
        # Score: scan_efficiency_score is already 0-100, use it directly as the
        # model selection criterion.  It incorporates coverage, maintenance, speed, safety.
        return {
            'env_steps': self.global_step,
            'score': float(np.mean(scores)),
            'reward_mean': float(np.mean(rewards)),
            'coverage_mean': float(np.mean(evers)),
            'ever_seen_fraction': float(np.mean(evers)),
            'ever_seen_min': float(np.min(evers)),
            'maintained_fraction': float(np.mean(mains)),
            'scan_efficiency_score': float(np.mean(scores)),
        }

    def _log_eval_row(self, eval_stats: Dict[str, float]):
        with self._safe_open(self.eval_csv, 'a') as f:
            csv.DictWriter(f, fieldnames=self._EVAL_FIELDS).writerow(eval_stats)

    def _log_train_row(self, info: Dict):
        row = {}
        for k in self._TRAIN_FIELDS:
            if k == 'env_steps':
                row[k] = self.global_step
            elif k == 'episode':
                row[k] = self.episode_idx
            elif k == 'reward_mean':
                row[k] = info.get('reward_mean', 0.0)
            elif k == 'coverage_mean':
                row[k] = info.get('ever_seen_fraction', 0.0)
            else:
                row[k] = info.get(k, 0.0)
        with self._safe_open(self.train_csv, 'a') as f:
            writer = csv.DictWriter(f, fieldnames=self._TRAIN_FIELDS)
            writer.writerow(row)

