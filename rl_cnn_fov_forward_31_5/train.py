from __future__ import annotations

import os
import re
import sys
from pathlib import Path

os.environ['PYTHONUNBUFFERED'] = '1'
sys.stdout.reconfigure(line_buffering=True)

import torch

from env_swarm_polygon import EnvConfig, SwarmSearchPolygonEnv
from ppo_trainer import PPOConfig, PPOTrainer


def _to_posix_if_wsl(path: str) -> str:
    if os.name != 'nt':
        m = re.match(r'[\\/]{2,}wsl[.$\\/].*?[\\/]([^\\/]+)([\\/].*)', path)
        if m:
            return m.group(2).replace('\\', '/')
    return path


ROOT = Path(__file__).resolve().parent
SCRIPT_DIR = _to_posix_if_wsl(str(ROOT))
OUTPUT_DIR = ROOT / 'outputs'

RESUME = False
RESUME_PATH = os.path.join(SCRIPT_DIR, 'outputs', 'models', 'latest_checkpoint.pt')


def make_env_cfg() -> EnvConfig:
    return EnvConfig(
        max_agents=5,
        min_active_agents=5,
        randomize_active_agents=False,
        allow_mid_episode_failures=False,
        failure_probability_per_second=0.0,
        decision_hz=10.0,
        episode_seconds=100.0,
        target_first_cover_fraction=0.98,
        world_w=40.0,
        world_h=40.0,
        polygon_num_vertices=12,
        polygon_target_area=900.0,
        polygon_margin=1.5,
        grid_resolution_m=0.5,
        local_map_size=32,
        critic_downsample=8,
        max_speed=3.5,
        max_turn_rate_deg=180.0,
        max_strafe_ratio=0.30,
        drone_safety_radius=4.5,
        wall_safety_margin=1.5,
        coverage_wall_margin=1.5,
        spawn_pair_distance=8.0,
        spawn_wall_margin=2.0,
        sensor_forward_range_m=0.4,
        scan_range_m=2.6,
        scan_fov_deg=100.0, #
        forward_sensor_fov_deg=60.0, #
        decay_reset_seconds=30.0,
        maintained_threshold=0.45,
    )


def make_ppo_cfg() -> PPOConfig:
    return PPOConfig(
        total_env_steps=3_500_000,
        rollout_steps=768,
        gamma=0.995,
        gae_lambda=0.95,
        clip_range=0.2,
        learning_rate=2.0e-4,
        update_epochs=6,
        minibatch_size=512,
        entropy_coef=0.01,
        value_coef=0.5,
        frontier_aux_coef=0.5,
        max_grad_norm=0.8,
        eval_every=25_000,
        save_every=25_000,
        device='cuda' if torch.cuda.is_available() else 'cpu',
    )


def main():
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    print(f'[train] CUDA available: {torch.cuda.is_available()}', flush=True)
    if torch.cuda.is_available():
        print(f'[train] GPU: {torch.cuda.get_device_name(0)}', flush=True)
    print(f'[train] Using device: {device}', flush=True)

    env = SwarmSearchPolygonEnv(make_env_cfg())
    trainer = PPOTrainer(env, make_ppo_cfg(), out_dir=str(OUTPUT_DIR))
    if RESUME and os.path.exists(RESUME_PATH):
        print(f'[train] resuming from {RESUME_PATH}', flush=True)
        trainer.load_checkpoint(RESUME_PATH)
    trainer.train()


if __name__ == '__main__':
    main()
