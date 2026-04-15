from __future__ import annotations

import csv
import os

import torch

from env_swarm_coverage import EnvConfig, SwarmCoverageEnv
from model import ActorCritic
from visualization import render_drone_knowledge_frame, render_global_frame, save_gif


def load_model(model_path: str, env: SwarmCoverageEnv, device: str = 'cpu') -> ActorCritic:
    map_spec, self_dim = env.actor_obs_spec
    model = ActorCritic(map_spec[0], map_spec[1], self_dim, env.centralized_state_dim).to(device)
    ckpt = torch.load(model_path, map_location=device)
    model.load_state_dict(ckpt['model'])
    model.eval()
    return model


def main():
    os.makedirs('outputs/videos', exist_ok=True)
    os.makedirs('outputs/eval', exist_ok=True)

    env = SwarmCoverageEnv(EnvConfig())
    model_path = 'outputs/models/best_model.pt'
    if not os.path.exists(model_path):
        model_path = 'outputs/models/latest_checkpoint.pt'
    if not os.path.exists(model_path):
        model_path = 'outputs/models/last_model.pt'
    if not os.path.exists(model_path):
        raise FileNotFoundError('No saved model found in outputs/models')

    device = 'cpu'
    model = load_model(model_path, env, device=device)

    obs, info = env.reset(seed=123)
    global_frames = []
    drone_frames = [[] for _ in range(env.n_agents)]
    rows = []

    for t in range(env.max_steps):
        maps = torch.as_tensor([o['maps'] for o in obs], dtype=torch.float32, device=device)
        self_s = torch.as_tensor([o['self_state'] for o in obs], dtype=torch.float32, device=device)
        with torch.no_grad():
            dist = model.actor.distribution(maps, self_s)
            actions = torch.tanh(dist.mean).cpu().numpy()
        obs, rewards, terminated, truncated, info = env.step(actions)
        state = env.render_state()
        global_frames.append(render_global_frame(state, env.cfg.world_w, env.cfg.world_h, title='Global mission'))
        for i in range(env.n_agents):
            drone_frames[i].append(render_drone_knowledge_frame(state, i, env.cfg.world_w, env.cfg.world_h, title=f'Drone {i}'))
        rows.append({'step': t, **info})
        if terminated or truncated:
            break

    save_gif(global_frames, 'outputs/videos/global_mission.gif', fps=10)
    for i in range(env.n_agents):
        save_gif(drone_frames[i], f'outputs/videos/drone_{i}_knowledge.gif', fps=10)

    out_csv = 'outputs/eval/eval_metrics.csv'
    with open(out_csv, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print('Saved evaluation GIFs and CSV.')


if __name__ == '__main__':
    main()
