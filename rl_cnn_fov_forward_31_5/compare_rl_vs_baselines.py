"""Compare trained RL policy against FrontierHeuristic and Lawnmower baselines.

Outputs are saved under outputs/comparison/.
"""
from __future__ import annotations

import csv
import os
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import torch

from baseline_controllers import FrontierHeuristicController, LawnmowerHeuristicController
from env_swarm_polygon import SwarmSearchPolygonEnv
from model import ActorCritic
from train import OUTPUT_DIR, make_env_cfg

ROOT = Path(__file__).resolve().parent
COMPARISON_DIR = ROOT / 'outputs' / 'comparison'


def load_model(model_path: Path, env: SwarmSearchPolygonEnv, device: str = 'cpu') -> ActorCritic:
    model = ActorCritic(env.actor_obs_spec, env.critic_obs_spec).to(device)
    ckpt = torch.load(model_path, map_location=device)
    model.load_state_dict(ckpt['model'])
    model.eval()
    return model


def evaluate_policy(env: SwarmSearchPolygonEnv, controller, num_episodes: int = 10,
                    deterministic: bool = True) -> dict[str, float]:
    reward_means, evers, maintains, scan_scores, t80s, t95s = [], [], [], [], [], []
    uncovereds, stales = [], []
    for ep in range(num_episodes):
        obs, info = env.reset(seed=10_000 + ep)
        if hasattr(controller, 'reset'):
            controller.reset(env)
        total = 0.0
        for _ in range(env.max_steps):
            if isinstance(controller, ActorCritic):
                obs_t = {k: torch.as_tensor(v, dtype=torch.float32) for k, v in obs.items()}
                with torch.no_grad():
                    actions_t, _ = controller.actor.act(
                        obs_t['maps'], obs_t['self_state'], obs_t['neighbor_state'], obs_t['neighbor_mask'],
                        deterministic=deterministic,
                    )
                actions = actions_t.cpu().numpy()
            else:
                actions = controller.act(env)
            obs, rewards, terminated, truncated, info = env.step(actions)
            active = max(1.0, float(np.sum(obs['active_mask'])))
            total += float(np.sum(rewards) / active)
            if terminated or truncated:
                break
        reward_means.append(total)
        evers.append(info['percent_covered_at_least_once'])
        maintains.append(info['maintained_fraction'])
        scan_scores.append(info['scan_efficiency_score'])
        uncovereds.append(info['uncovered_fraction'])
        stales.append(info['stale_fraction'])
        t80s.append(info['time_to_80_coverage'])
        t95s.append(info['time_to_95_coverage'])
    return {
        'reward_mean': float(np.mean(reward_means)),
        'reward_std': float(np.std(reward_means)),
        'percent_covered': float(np.mean(evers)),
        'maintained': float(np.mean(maintains)),
        'uncovered_fraction': float(np.mean(uncovereds)),
        'stale_fraction': float(np.mean(stales)),
        'scan_efficiency': float(np.mean(scan_scores)),
        'time_to_80': float(np.mean([t for t in t80s if t >= 0])) if any(t >= 0 for t in t80s) else -1,
        'time_to_95': float(np.mean([t for t in t95s if t >= 0])) if any(t >= 0 for t in t95s) else -1,
    }


def main():
    os.makedirs(COMPARISON_DIR, exist_ok=True)
    env = SwarmSearchPolygonEnv(make_env_cfg())
    device = 'cuda' if torch.cuda.is_available() else 'cpu'

    model_path = None
    for name in ['best_model.pt', 'best_so_far.pt', 'latest_checkpoint.pt']:
        p = OUTPUT_DIR / 'models' / name
        if p.exists():
            model_path = p
            break
    rl_model = None if model_path is None else load_model(model_path, env, device)
    if rl_model is None:
        print('[compare] No trained model found — evaluating baselines only.')
    else:
        print(f'[compare] Loading RL model from {model_path}')

    frontier = FrontierHeuristicController(env.cfg.max_speed, env.cfg.world_w, env.cfg.world_h)
    lawnmower = LawnmowerHeuristicController(env.cfg.max_speed, env.cfg.world_w, env.cfg.world_h, stripe_spacing=4.0)

    results = {}
    controllers = {'Frontier heuristic': frontier, 'Lawnmower heuristic': lawnmower}
    if rl_model is not None:
        controllers['RL CNN'] = rl_model
    for name, ctrl in controllers.items():
        print(f'[compare] evaluating {name}...')
        results[name] = evaluate_policy(env, ctrl, num_episodes=10)
        r = results[name]
        print(f"    covered={r['percent_covered']:.1f}% maintained={r['maintained']:.3f} "
              f"uncovered={r['uncovered_fraction']:.3f} stale={r['stale_fraction']:.3f} "
              f"score={r['scan_efficiency']:.1f}")

    csv_path = COMPARISON_DIR / 'comparison_results.csv'
    fields = ['method'] + list(next(iter(results.values())).keys())
    with open(csv_path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for name, stats in results.items():
            writer.writerow({'method': name, **stats})

    methods = list(results.keys())
    metrics = ['percent_covered', 'maintained', 'uncovered_fraction', 'stale_fraction', 'scan_efficiency']
    fig, axes = plt.subplots(1, len(metrics), figsize=(4.3 * len(metrics), 5))
    if len(metrics) == 1:
        axes = [axes]
    for ax, metric in zip(axes, metrics):
        vals = [results[m][metric] for m in methods]
        bars = ax.bar(methods, vals)
        ax.set_title(metric.replace('_', ' ').title())
        for tick in ax.get_xticklabels():
            tick.set_rotation(20)
        for bar, v in zip(bars, vals):
            y = bar.get_height() + (0.5 if metric == 'percent_covered' else 0.01)
            ax.text(bar.get_x() + bar.get_width() / 2, y, f'{v:.3f}' if metric != 'percent_covered' else f'{v:.1f}',
                    ha='center', va='bottom', fontsize=8)
    fig.suptitle('RL vs baselines on concave polygon coverage maintenance', fontsize=14)
    fig.tight_layout()
    fig.savefig(str(COMPARISON_DIR / 'comparison_chart.png'), dpi=150)
    plt.close(fig)
    print(f'[compare] Saved outputs to {COMPARISON_DIR}')


if __name__ == '__main__':
    main()
