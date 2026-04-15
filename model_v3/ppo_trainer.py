from __future__ import annotations

import csv
import os
import re
import stat
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, IO, List

import numpy as np
import torch
import torch.nn.functional as F
from torch.optim import Adam

from env_swarm_coverage import SwarmCoverageEnv
from model import ActorCritic


@dataclass
class PPOConfig:
    """Configuration for PPO training."""
    total_env_steps: int = 3_000_000
    rollout_steps: int = 1024
    gamma: float = 0.99 # discount factor for rewards
    gae_lambda: float = 0.95 # GAE lambda for advantage estimation
    clip_range: float = 0.2 # clipping range for PPO surrogate objective
    learning_rate: float = 3e-4
    update_epochs: int = 8
    minibatch_size: int = 512
    entropy_coef: float = 0.01 # coefficient for entropy regularization
    value_coef: float = 0.5 # coefficient for value function loss
    max_grad_norm: float = 0.5
    eval_every: int = 20_000
    save_every: int = 20_000
    device: str = 'cpu'


class RolloutBuffer:
    def __init__(self):
        self.maps = []
        self.self_states = []
        self.central_states = []
        self.actions = []
        self.log_probs = []
        self.rewards = []
        self.dones = []
        self.values = []

    def add(self, **kwargs):
        for key, value in kwargs.items():
            getattr(self, key).append(value)

    def clear(self):
        for key in ['maps', 'self_states', 'central_states', 'actions', 'log_probs', 'rewards', 'dones', 'values']:
            getattr(self, key).clear()


class PPOTrainer:
    def __init__(self, env: SwarmCoverageEnv, cfg: PPOConfig, out_dir: str):
        self.env = env
        self.cfg = cfg
        self.device = torch.device(cfg.device)
        map_spec, self_dim = env.actor_obs_spec
        critic_dim = env.centralized_state_dim
        self.model = ActorCritic(map_spec[0], map_spec[1], self_dim, critic_dim).to(self.device)
        self.optimizer = Adam(self.model.parameters(), lr=cfg.learning_rate)

        # ---- resolve out_dir to an absolute POSIX path next to this script ----
        _script_dir = str(Path(__file__).resolve().parent)
        _script_dir = self._to_posix_if_wsl(_script_dir)
        out_dir = os.path.join(_script_dir, out_dir)

        self.out_dir = out_dir
        os.makedirs(out_dir, exist_ok=True)
        os.makedirs(os.path.join(out_dir, 'models'), exist_ok=True)
        os.makedirs(os.path.join(out_dir, 'logs'), exist_ok=True)
        os.makedirs(os.path.join(out_dir, 'videos'), exist_ok=True)

        self.global_step = 0
        self.best_eval_score = -1e18
        self.best_eval_step = 0
        self.last_eval_score = None
        self._train_start_time = time.time()

        self.train_csv = os.path.join(out_dir, 'logs', 'train_metrics.csv')
        self.loss_csv = os.path.join(out_dir, 'logs', 'ppo_losses.csv')
        self.eval_csv = os.path.join(out_dir, 'logs', 'eval_metrics.csv')
        self._init_csvs()

    # ------------------------------------------------------------------
    # Filesystem helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _to_posix_if_wsl(path: str) -> str:
        """Convert ``\\\\wsl.localhost\\Distro\\…`` or ``\\\\wsl$\\Distro\\…``
        UNC paths into native POSIX ``/home/…`` paths when running inside WSL.
        This avoids PermissionError caused by Windows' UNC file-system bridge."""
        if os.name != 'nt':
            # e.g.  \\wsl.localhost\Ubuntu-22.04\home\user  ->  /home/user
            m = re.match(r'[\\/]{2,}wsl[.$\\/].*?[\\/]([^\\/]+)([\\/].*)', path)
            if m:
                return m.group(2).replace('\\', '/')
        return path

    _MAX_OPEN_RETRIES = 3
    _RETRY_DELAY = 0.5  # seconds

    @staticmethod
    def _safe_open(filepath: str, mode: str = 'a') -> IO[str]:
        """Open *filepath* for writing with retries for transient locks."""
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
                        os.chmod(filepath,
                                 stat.S_IRUSR | stat.S_IWUSR | stat.S_IRGRP | stat.S_IROTH)
                    except OSError:
                        pass
                time.sleep(PPOTrainer._RETRY_DELAY * (attempt + 1))

        # last resort: write to a fallback file next to the original
        fallback = filepath + f'.{int(time.time())}.tmp'
        print(f'[warning] cannot write {filepath} ({last_err}); '
              f'falling back to {fallback}')
        return open(fallback, mode, newline='', encoding='utf-8')

    def _init_csvs(self):
        with self._safe_open(self.train_csv, 'w') as f:
            writer = csv.DictWriter(f, fieldnames=[
                'env_steps','episode','reward_mean','coverage_mean','ever_seen_fraction','maintained_fraction',
                'new_area_delta','time_to_50_coverage','time_to_80_coverage','time_to_95_coverage',
                'overlap_ratio_mean','collision_penalty_mean','min_inter_drone_dist','scan_efficiency_score'
            ])
            writer.writeheader()
        with self._safe_open(self.loss_csv, 'w') as f:
            writer = csv.DictWriter(f, fieldnames=['env_steps','policy_loss','value_loss','entropy','total_loss'])
            writer.writeheader()
        with self._safe_open(self.eval_csv, 'w') as f:
            writer = csv.DictWriter(f, fieldnames=['env_steps','score','reward_mean','coverage_mean','ever_seen_fraction','scan_efficiency_score'])
            writer.writeheader()

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

    def _stack_actor_obs(self, obs: List[Dict[str, np.ndarray]]):
        maps = torch.as_tensor(np.stack([o['maps'] for o in obs], axis=0), dtype=torch.float32, device=self.device)
        self_s = torch.as_tensor(np.stack([o['self_state'] for o in obs], axis=0), dtype=torch.float32, device=self.device)
        return maps, self_s

    def train(self):
        buffer = RolloutBuffer()
        obs, info = self.env.reset(seed=0)
        episode_idx = 0
        print(f'[train] starting: total_env_steps={self.cfg.total_env_steps} rollout_steps={self.cfg.rollout_steps} n_agents={self.env.n_agents} device={self.cfg.device}')

        while self.global_step < self.cfg.total_env_steps:
            for _ in range(self.cfg.rollout_steps):
                maps_t, self_t = self._stack_actor_obs(obs)
                central_state = torch.as_tensor(self.env.get_centralized_state(), dtype=torch.float32, device=self.device).unsqueeze(0)
                with torch.no_grad():
                    actions_t, log_probs_t = self.model.actor.act(maps_t, self_t)
                    value_t = self.model.critic(central_state).repeat(self.env.n_agents)
                actions = actions_t.cpu().numpy()
                next_obs, rewards, terminated, truncated, info = self.env.step(actions)
                done_flag = terminated or truncated
                buffer.add( # store data for all agents at this step
                    maps=maps_t.cpu().numpy(),
                    self_states=self_t.cpu().numpy(),
                    central_states=np.repeat(central_state.cpu().numpy(), self.env.n_agents, axis=0),
                    actions=actions,
                    log_probs=log_probs_t.cpu().numpy(),
                    rewards=rewards.copy(),
                    dones=np.full(self.env.n_agents, float(done_flag), dtype=np.float32),
                    values=value_t.cpu().numpy(),
                )
                self.global_step += self.env.n_agents
                obs = next_obs
                if done_flag:
                    episode_idx += 1
                    self._log_train_row(episode_idx, info)
                    if episode_idx % 10 == 0:
                        print(f"[train] step={self.global_step} episode={episode_idx} reward={info.get('reward_mean',0.0):.3f} coverage={info.get('coverage_mean',0.0):.3f} ever_seen={info.get('ever_seen_fraction',0.0):.3f} new_area={info.get('new_area_delta',0.0):.3f} overlap={info.get('overlap_ratio_mean',0.0):.3f} collision={info.get('collision_penalty_mean',0.0):.3f} score={info.get('scan_efficiency_score',0.0):.3f}")
                    obs, info = self.env.reset(seed=episode_idx)
                if self.global_step >= self.cfg.total_env_steps:
                    break

            last_state = torch.as_tensor(self.env.get_centralized_state(), dtype=torch.float32, device=self.device).unsqueeze(0)
            with torch.no_grad():
                last_value = self.model.critic(last_state).repeat(self.env.n_agents).cpu().numpy()
            loss_dict = self._ppo_update(buffer, last_value)
            buffer.clear()
            self._log_loss_row(loss_dict)
            self.save_checkpoint('latest_checkpoint.pt')

            if self.global_step % self.cfg.save_every < self.env.n_agents:
                self.save_checkpoint(f'checkpoint_{self.global_step}.pt')

            if self.global_step % self.cfg.eval_every < self.env.n_agents:
                eval_stats = self.evaluate(num_episodes=5)
                self._log_eval_row(eval_stats)
                eval_score = float(eval_stats['score'])
                delta = None if self.last_eval_score is None else (eval_score - self.last_eval_score)
                self.last_eval_score = eval_score
                elapsed = time.time() - self._train_start_time
                sps = self.global_step / max(1e-6, elapsed)
                delta_s = 'n/a' if delta is None else f'{delta:+.3f}'
                print(f"[eval] step={self.global_step} score={eval_score:.3f} (Δ {delta_s}) reward={eval_stats['reward_mean']:.3f} coverage={eval_stats['coverage_mean']:.3f} ever_seen={eval_stats['ever_seen_fraction']:.3f} best={self.best_eval_score:.3f} sps={sps:.1f}")
                if eval_score > self.best_eval_score:
                    self.best_eval_score = eval_score
                    self.best_eval_step = self.global_step
                    self.save_checkpoint('best_model.pt')
                    self.save_checkpoint('best_so_far.pt')
                    print(f'[eval] new best saved at step={self.global_step} score={eval_score:.3f}')

        self.save_checkpoint('last_model.pt')
        print(f'[train] done. final_step={self.global_step} best_score={self.best_eval_score:.3f} at step={self.best_eval_step}')

    def _ppo_update(self, buffer: RolloutBuffer, last_value: np.ndarray) -> Dict[str,float]:
        rewards = np.asarray(buffer.rewards, dtype=np.float32)
        dones = np.asarray(buffer.dones, dtype=np.float32)
        values = np.asarray(buffer.values, dtype=np.float32)
        T, N = rewards.shape
        advantages = np.zeros_like(rewards)
        last_gae = np.zeros(N, dtype=np.float32)
        next_values = last_value.astype(np.float32)
        for t in reversed(range(T)):
            nonterminal = 1.0 - dones[t]
            delta = rewards[t] + self.cfg.gamma * next_values * nonterminal - values[t]
            last_gae = delta + self.cfg.gamma * self.cfg.gae_lambda * nonterminal * last_gae
            advantages[t] = last_gae
            next_values = values[t]
        returns = advantages + values

        maps = torch.as_tensor(np.concatenate(buffer.maps, axis=0), dtype=torch.float32, device=self.device)
        self_states = torch.as_tensor(np.concatenate(buffer.self_states, axis=0), dtype=torch.float32, device=self.device)
        central_states = torch.as_tensor(np.concatenate(buffer.central_states, axis=0), dtype=torch.float32, device=self.device)
        actions = torch.as_tensor(np.concatenate(buffer.actions, axis=0), dtype=torch.float32, device=self.device) #
        old_log_probs = torch.as_tensor(np.concatenate(buffer.log_probs, axis=0), dtype=torch.float32, device=self.device)
        advantages_t = torch.as_tensor(advantages.reshape(-1), dtype=torch.float32, device=self.device)
        returns_t = torch.as_tensor(returns.reshape(-1), dtype=torch.float32, device=self.device)
        advantages_t = (advantages_t - advantages_t.mean()) / (advantages_t.std() + 1e-8)

        total_policy = total_value = total_entropy = 0.0
        count = 0
        idxs = np.arange(actions.shape[0])
        for _ in range(self.cfg.update_epochs):
            np.random.shuffle(idxs)
            for start in range(0, len(idxs), self.cfg.minibatch_size):
                mb = idxs[start:start+self.cfg.minibatch_size]
                new_log_prob, entropy = self.model.actor.evaluate_actions(maps[mb], self_states[mb], actions[mb])
                values_pred = self.model.critic(central_states[mb])
                ratio = torch.exp(new_log_prob - old_log_probs[mb])
                s1 = ratio * advantages_t[mb]
                s2 = torch.clamp(ratio, 1.0 - self.cfg.clip_range, 1.0 + self.cfg.clip_range) * advantages_t[mb]
                policy_loss = -torch.min(s1, s2).mean()
                value_loss = F.mse_loss(values_pred, returns_t[mb])
                entropy_mean = entropy.mean()
                total_loss = policy_loss + self.cfg.value_coef * value_loss - self.cfg.entropy_coef * entropy_mean

                self.optimizer.zero_grad()
                total_loss.backward()
                torch.nn.utils.clip_grad_norm_(self.model.parameters(), self.cfg.max_grad_norm)
                self.optimizer.step()

                total_policy += float(policy_loss.item())
                total_value += float(value_loss.item())
                total_entropy += float(entropy_mean.item())
                count += 1
        return {
            'policy_loss': total_policy / max(1, count),
            'value_loss': total_value / max(1, count),
            'entropy': total_entropy / max(1, count),
            'total_loss': (total_policy + total_value) / max(1, count),
        }

    def evaluate(self, num_episodes: int = 5) -> Dict[str,float]:
        rewards = []
        coverages = []
        evers = []
        scores = []
        for ep in range(num_episodes):
            obs, info = self.env.reset(seed=10_000 + ep)
            total = 0.0
            for _ in range(self.env.max_steps):
                maps_t, self_t = self._stack_actor_obs(obs)
                with torch.no_grad():
                    dist = self.model.actor.distribution(maps_t, self_t)
                    actions = torch.tanh(dist.mean).cpu().numpy()
                obs, r, terminated, truncated, info = self.env.step(actions)
                total += float(np.mean(r))
                if terminated or truncated:
                    break
            rewards.append(total)
            coverages.append(float(info.get('coverage_mean',0.0)))
            evers.append(float(info.get('ever_seen_fraction',0.0)))
            scores.append(float(info.get('scan_efficiency_score',0.0)))
        return {
            'env_steps': self.global_step,
            'score': float(np.mean(scores) + 0.5 * np.mean(evers)),
            'reward_mean': float(np.mean(rewards)),
            'coverage_mean': float(np.mean(coverages)),
            'ever_seen_fraction': float(np.mean(evers)),
            'scan_efficiency_score': float(np.mean(scores)),
        }

    def _log_train_row(self, episode: int, info: Dict[str,float]):
        with self._safe_open(self.train_csv, 'a') as f:
            writer = csv.DictWriter(f, fieldnames=['env_steps','episode','reward_mean','coverage_mean','ever_seen_fraction','maintained_fraction','new_area_delta','time_to_50_coverage','time_to_80_coverage','time_to_95_coverage','overlap_ratio_mean','collision_penalty_mean','min_inter_drone_dist','scan_efficiency_score'])
            row = {'env_steps': self.global_step, 'episode': episode}
            for k in writer.fieldnames:
                if k not in row:
                    row[k] = info.get(k, 0.0)
            writer.writerow(row)

    def _log_loss_row(self, loss_dict: Dict[str,float]):
        with self._safe_open(self.loss_csv, 'a') as f:
            writer = csv.DictWriter(f, fieldnames=['env_steps','policy_loss','value_loss','entropy','total_loss'])
            writer.writerow({'env_steps': self.global_step, **loss_dict})

    def _log_eval_row(self, stats: Dict[str,float]):
        with self._safe_open(self.eval_csv, 'a') as f:
            writer = csv.DictWriter(f, fieldnames=['env_steps','score','reward_mean','coverage_mean','ever_seen_fraction','scan_efficiency_score'])
            writer.writerow(stats)
