from __future__ import annotations

import os
import re
from pathlib import Path

from env_swarm_coverage import EnvConfig, SwarmCoverageEnv
from ppo_trainer import PPOConfig, PPOTrainer


def _to_posix_if_wsl(path: str) -> str:
    """Convert UNC ``\\\\wsl.localhost\\...`` paths to native POSIX paths inside WSL."""
    if os.name != 'nt':
        m = re.match(r'[\\/]{2,}wsl[.$\\/].*?[\\/]([^\\/]+)([\\/].*)', path)
        if m:
            return m.group(2).replace('\\', '/')
    return path


_SCRIPT_DIR = _to_posix_if_wsl(str(Path(__file__).resolve().parent))

RESUME = True
RESUME_PATH = os.path.join(_SCRIPT_DIR, 'outputs', 'models', 'latest_checkpoint.pt')


def main():
    env_cfg = EnvConfig(
        n_agents=4,
        world_w=20.0,
        world_h=20.0,
        decision_hz=10.0,
        episode_seconds=90.0,
        drone_safety_radius=0.50,
        wall_safety_margin=0.05,
        decay_reset_seconds=60.0,
        heading_fov_deg=80.0,
        sensor_forward_range=0.50,
    )
    ppo_cfg = PPOConfig(
        total_env_steps=3_500_000,
        rollout_steps=1024,
        eval_every=20_000,
        save_every=20_000,
        device='cpu',
    )
    env = SwarmCoverageEnv(env_cfg)
    trainer = PPOTrainer(env, ppo_cfg, out_dir='outputs')
    if RESUME and os.path.exists(RESUME_PATH):
        print(f'[train] resuming from {RESUME_PATH}')
        trainer.load_checkpoint(RESUME_PATH)
    trainer.train()


if __name__ == '__main__':
    main()
