from __future__ import annotations

import csv
import os
from pathlib import Path

import torch

from env_swarm_polygon import SwarmSearchPolygonEnv
from model import ActorCritic
from train_fast import make_env_cfg
from visualization import render_coverage_summary, render_global_frame, save_gif

ROOT = Path(__file__).resolve().parent

def load_model(model_path: str, env: SwarmSearchPolygonEnv, device: str = 'cpu') -> ActorCritic:
    model = ActorCritic(env.actor_obs_spec, env.critic_obs_spec).to(device)
    ckpt = torch.load(model_path, map_location=device)
    model.load_state_dict(ckpt['model'])
    model.eval()
    return model

def find_best_model() -> str:
    for name in ['best_model.pt', 'model_100coverage.pt','best_so_far.pt', 'latest_checkpoint.pt', 'last_model.pt']:
        p = ROOT / 'outputs' / 'models' / name
        if p.exists():
            return str(p)
    raise FileNotFoundError('No saved model found in outputs/models')


def eval_vis():
    out_vid = ROOT / 'outputs' / 'videos'
    out_eval = ROOT / 'outputs' / 'eval'
    os.makedirs(out_vid, exist_ok=True)
    os.makedirs(out_eval, exist_ok=True)

    env = SwarmSearchPolygonEnv(make_env_cfg())
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    model = load_model(find_best_model(), env, device=device)

    obs, info = env.reset(seed=123)
    global_frames = []
    summary_frames = []
    rows = []

    for t in range(env.max_steps):
        obs_t = {k: torch.as_tensor(v, dtype=torch.float32, device=device) for k, v in obs.items()}
        with torch.no_grad():
            actions_t, _ = model.actor.act(
                obs_t['maps'], obs_t['self_state'], obs_t['neighbor_state'], obs_t['neighbor_mask'],
                deterministic=True,
            )
        obs, rewards, terminated, truncated, info = env.step(actions_t.cpu().numpy())
        state = env.render_state()

        if t % 5 == 0 or terminated or truncated:
            title = (
                f"Step {t} | covered once: {info.get('percent_covered_at_least_once', 0):.1f}% | "
                f"maintained: {info.get('maintained_fraction', 0):.3f}"
            )
            global_frames.append(render_global_frame(state, env.cfg.world_w, env.cfg.world_h, title=title))
            summary_frames.append(render_coverage_summary(state, env.cfg.world_w, env.cfg.world_h, info, title=f'Step {t}'))

        rows.append({'step': t, **info})
        if terminated or truncated:
            break

    save_gif(global_frames, str(out_vid / 'global_mission.gif'), fps=10)
    save_gif(summary_frames, str(out_vid / 'coverage_summary.gif'), fps=6)

    csv_path = out_eval / 'episode_metrics.csv'
    if rows:
        with open(csv_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
    print(f'[eval] saved outputs to {out_vid} and {csv_path}')


if __name__ == '__main__':
    eval_vis()
