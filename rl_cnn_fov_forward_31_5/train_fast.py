"""Fast training script using vectorized (parallel) environments."""
from __future__ import annotations

import multiprocessing as mp
import os
import sys

os.environ['PYTHONUNBUFFERED'] = '1'
sys.stdout.reconfigure(line_buffering=True)

import torch

from env_swarm_polygon import EnvConfig
from ppo_trainer_vec import PPOConfig, PPOTrainerVec
from pathlib import Path

ROOT = Path(__file__).resolve().parent
OUTPUT_DIR = ROOT / 'outputs'
RESUME = False


def make_env_cfg() -> EnvConfig:
    return EnvConfig(
        max_agents=4,
        min_active_agents=4,
        randomize_active_agents=False,
        allow_mid_episode_failures=False,
        failure_probability_per_second=0.0,
        decision_hz=10.0,
        episode_seconds=180.0,         # longer for 60° FOV
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
        max_turn_rate_deg=360.0,
        max_strafe_ratio=0.30,
        drone_safety_radius=2.0,
        wall_safety_margin=1.5,
        coverage_wall_margin=1.5,  # 1.5m dead zone from wall — not coverable
        spawn_pair_distance=6.0,
        spawn_wall_margin=3.0,
        sensor_forward_range_m=0.4,
        scan_range_m=4.0,
        scan_fov_deg=60.0,
        forward_sensor_fov_deg=60.0,
        corner_scan_distance=5.0,
        trajectory_heatmap_decay=0.97,
        decay_reset_seconds=120.0,
        maintained_threshold=0.45,
    )


def make_ppo_cfg() -> PPOConfig:
    return PPOConfig(
        total_env_steps=2_500_000,     # more steps for harder task
        num_workers= 30,
        num_envs= 5,
        rollout_steps=256,               # longer rollout for 1800-step episodes
        gamma=0.995,
        gae_lambda=0.95,
        clip_range=0.2,
        learning_rate=2.5e-4,
        update_epochs=6,
        minibatch_size=2048,     # larger minibatch to match more data
        entropy_coef=0.02,       # was 0.01 — more exploration to escape circling
        value_coef=0.5,
        frontier_aux_coef=0.5,
        max_grad_norm=0.8,
        eval_every=25_000,
        save_every=25_000,
        device='cuda' if torch.cuda.is_available() else 'cpu',
    )


def find_best_checkpoint() -> str | None:
    """Find the best saved model checkpoint to resume from."""
    models_dir = OUTPUT_DIR / 'models'
    if not models_dir.exists():
        return None
    # Priority: best_model.pt > model_*coverage*.pt > latest checkpoint
    candidates = [
        models_dir / 'latest_checkpoint.pt',
        models_dir / 'best_model.pt',
    ]
    for c in candidates:
        if c.exists():
            return str(c)
    # Look for model_*coverage* files (e.g. model_100coverage.pt)
    coverage_models = sorted(models_dir.glob('model_*coverage*.pt'), reverse=True)
    if coverage_models:
        return str(coverage_models[0])
    # Any .pt file sorted by modification time (most recent)
    all_pts = sorted(models_dir.glob('*.pt'), key=lambda p: p.stat().st_mtime, reverse=True)
    if all_pts:
        return str(all_pts[0])
    return None


def main():
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    print(f'[train-fast] CUDA: {torch.cuda.is_available()}', flush=True)
    if torch.cuda.is_available():
        print(f'[train-fast] GPU: {torch.cuda.get_device_name(0)}', flush=True)
    ppo_cfg = make_ppo_cfg()
    print(f'[train-fast] device={device}, num_workers={ppo_cfg.num_workers}, num_envs={ppo_cfg.num_envs}', flush=True)

    trainer = PPOTrainerVec(make_env_cfg(), ppo_cfg, out_dir=str(OUTPUT_DIR))

    # Resume from best checkpoint if available
    resume_path = find_best_checkpoint()
    if resume_path and RESUME:
        print(f'[train-fast] Resuming from checkpoint: {resume_path}', flush=True)
        try:
            trainer.load_checkpoint(resume_path)
            print(f'[train-fast] Resumed at global_step={trainer.global_step}, '
                  f'best_eval_score={trainer.best_eval_score:.4f}', flush=True)
        except Exception as e:
            print(f'[train-fast] WARNING: Failed to load checkpoint: {e}', flush=True)
            print(f'[train-fast] Starting fresh training instead.', flush=True)
    else:
        print(f'[train-fast] No checkpoint found, starting fresh training.', flush=True)

    trainer.train()


if __name__ == '__main__':
    mp.set_start_method('spawn', force=True)
    main()


