"""
Visualize CNN input channels, intermediate feature maps, and final conv layer activations.
Run this to diagnose what the model "sees" at different stages of an episode.

Generates:
  outputs/eval/channels_step{N}_drone{D}.png — input channels for each drone at step N
  outputs/eval/features_step{N}_drone{D}.png — feature maps from each conv layer
  outputs/eval/channel_evolution.png — how channels change over time for one drone
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

import numpy as np
import torch
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from env_swarm_polygon import SwarmSearchPolygonEnv
from model import ActorCritic


# ─────────────────────────────────────────────────────────────────────────────
# Hook system: capture intermediate feature maps from CNN layers
# ─────────────────────────────────────────────────────────────────────────────

class FeatureMapCapture:
    """Register forward hooks on CNN layers to capture feature maps."""

    def __init__(self, model: ActorCritic):
        self.features = {}
        self.hooks = []
        # Hook into actor's MapEncoder conv blocks
        encoder = model.actor.encoder.net
        for idx, layer in enumerate(encoder):
            if hasattr(layer, 'conv'):  # ConvBlock
                hook = layer.register_forward_hook(self._make_hook(f'conv_{idx}'))
                self.hooks.append(hook)

    def _make_hook(self, name):
        def hook_fn(module, input, output):
            self.features[name] = output.detach().cpu()
        return hook_fn

    def clear(self):
        self.features.clear()

    def remove_hooks(self):
        for h in self.hooks:
            h.remove()
        self.hooks.clear()


# ─────────────────────────────────────────────────────────────────────────────
# Visualization functions
# ─────────────────────────────────────────────────────────────────────────────

CHANNEL_NAMES = [
    'Ch0: Terrain & Coverage\n(Master Map)'
]


def plot_input_channels(maps: np.ndarray, drone_id: int, step: int,
                        self_state: np.ndarray = None, save_path: str = None):
    """Plot all 6 input channels for one drone at one timestep.

    maps: (C, H, W) — the 6-channel local map crop for this drone
    """
    n_ch = maps.shape[0]
    fig, axes = plt.subplots(1, n_ch, figsize=(5 * n_ch, 5))
    if n_ch == 1:
        axes = [axes]
    else:
        axes = axes.flatten()

    for ch in range(n_ch):
        ax = axes[ch]
        # Use vmin=-1 so that -1.0 walls appear darker than 0.0 unvisited areas
        im = ax.imshow(maps[ch], origin='lower', cmap='viridis', vmin=-1, vmax=1.0)
        ax.set_title(CHANNEL_NAMES[ch], fontsize=10, fontweight='bold')
        ax.set_xlabel('x (pixels)')
        ax.set_ylabel('y (pixels)')
        fig.colorbar(im, ax=ax, fraction=0.046)
        # Mark center (drone position)
        cx, cy = maps.shape[2] // 2, maps.shape[1] // 2
        ax.plot(cx, cy, 'r+', markersize=12, markeredgewidth=2)
        # Stats
        ax.text(0.02, 0.98, f'mean={maps[ch].mean():.3f}\nmax={maps[ch].max():.3f}',
                transform=ax.transAxes, va='top', fontsize=8,
                bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

    # All 3 subplots are used now, no need to hide any

    fig.suptitle(f'Input Channels — Drone {drone_id}, Step {step}', fontsize=13, fontweight='bold')

    if self_state is not None:
        state_text = (
            f'pos=({self_state[0]:.2f}, {self_state[1]:.2f})  '
            f'vel=({self_state[2]:.2f}, {self_state[3]:.2f})  '
            f'edge_dist={self_state[6]:.3f}  progress={self_state[7]:.2f}\n'
            f'local_cov_mean={self_state[8]:.3f}  local_never={self_state[11]:.3f}  '
            f'corner_prox={self_state[16]:.3f}  traj_density={self_state[17]:.3f}'
        )
        fig.text(0.5, 0.01, state_text, ha='center', fontsize=9,
                 bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.9))

    fig.tight_layout(rect=(0, 0.04, 1, 0.95))
    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f'  Saved: {save_path}')
    plt.close(fig)


def plot_feature_maps(features: dict, drone_id: int, step: int, save_path: str = None):
    """Plot feature maps from each conv layer (show top-8 most active filters per layer)."""
    n_layers = len(features)
    if n_layers == 0:
        print('  No feature maps captured.')
        return

    fig = plt.figure(figsize=(16, 4 * n_layers))
    gs = gridspec.GridSpec(n_layers, 8, figure=fig, hspace=0.4, wspace=0.3)

    for layer_idx, (name, feat) in enumerate(features.items()):
        # feat: (batch, C, H, W) — take first sample (this drone)
        f = feat[0]  # (C, H, W)
        n_filters = f.shape[0]
        # Select top-8 most activated filters
        activations = f.mean(dim=(1, 2))  # mean activation per filter
        top_indices = torch.argsort(activations, descending=True)[:8]

        for col, fi in enumerate(top_indices):
            ax = fig.add_subplot(gs[layer_idx, col])
            fmap = f[fi].numpy()
            im = ax.imshow(fmap, origin='lower', cmap='inferno')
            ax.set_title(f'f{fi.item()}', fontsize=8)
            ax.axis('off')
            if col == 0:
                ax.set_ylabel(f'{name}\n({n_filters}ch, {f.shape[1]}×{f.shape[2]})',
                             fontsize=9, fontweight='bold')

    fig.suptitle(f'Feature Maps (Top-8 Active Filters) — Drone {drone_id}, Step {step}',
                 fontsize=13, fontweight='bold')
    if save_path:
        fig.savefig(save_path, dpi=120, bbox_inches='tight')
        print(f'  Saved: {save_path}')
    plt.close(fig)


def plot_channel_evolution(history: dict, drone_id: int, save_path: str = None):
    """Plot how each channel's statistics evolve over time for one drone.

    history: {step: maps_array} where maps_array is (C, H, W)
    """
    steps = sorted(history.keys())
    n_ch = history[steps[0]].shape[0]

    fig, axes = plt.subplots(n_ch, 1, figsize=(12, 7), sharex=True)
    if n_ch == 1:
        axes = [axes]

    for ch in range(n_ch):
        means = [float(history[s][ch].mean()) for s in steps]
        maxs = [float(history[s][ch].max()) for s in steps]
        nonzero_frac = [float((history[s][ch] > 0.01).mean()) for s in steps]

        ax = axes[ch]
        ax.plot(steps, means, 'b-', linewidth=1.5, label='Mean')
        ax.plot(steps, maxs, 'r--', linewidth=1, label='Max')
        ax.plot(steps, nonzero_frac, 'g:', linewidth=1, label='Non-zero frac')
        ax.set_ylabel(CHANNEL_NAMES[ch].replace('\n', ' '), fontsize=9)
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.set_ylim(-0.05, 1.05)

    if n_ch == 1:
        axes[0].set_xlabel('Step')
    else:
        axes[-1].set_xlabel('Step')
    fig.suptitle(f'Channel Statistics Over Time — Drone {drone_id}', fontsize=13, fontweight='bold')
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f'  Saved: {save_path}')
    plt.close(fig)


def plot_channel_snapshots_grid(history: dict, drone_id: int, save_path: str = None):
    """Plot all 6 channels at multiple timesteps as a grid (channels × time)."""
    steps = sorted(history.keys())
    n_steps = len(steps)
    n_ch = history[steps[0]].shape[0]

    fig, axes = plt.subplots(n_ch, n_steps, figsize=(3 * n_steps, 2.5 * n_ch))
    if n_ch == 1 and n_steps == 1:
        axes = np.array([[axes]])
    elif n_ch == 1:
        axes = axes[None, :]
    elif n_steps == 1:
        axes = axes[:, None]

    for col, step in enumerate(steps):
        for row in range(n_ch):
            ax = axes[row, col]
            # Use vmin=-1 to show walls, vmax=1.0 for Master Map
            im = ax.imshow(history[step][row], origin='lower', cmap='viridis', vmin=-1, vmax=1.0)
            ax.axis('off')
            if col == 0:
                ax.set_ylabel(CHANNEL_NAMES[row].replace('\n', ' '), fontsize=8)
            if row == 0:
                ax.set_title(f'Step {step}', fontsize=9, fontweight='bold')

    fig.suptitle(f'Channel Snapshots Over Time — Drone {drone_id}', fontsize=13, fontweight='bold')
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    if save_path:
        fig.savefig(save_path, dpi=120, bbox_inches='tight')
        print(f'  Saved: {save_path}')
    plt.close(fig)


def plot_last_conv_analysis(features: dict, step: int, save_path: str = None):
    """Analyze the LAST conv layer: show activation histogram and spatial attention map."""
    if not features:
        return
    # Get last conv layer
    last_name = list(features.keys())[-1]
    feat = features[last_name][0]  # (C, H, W)

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))

    # 1. Activation histogram
    ax = axes[0]
    vals = feat.numpy().flatten()
    ax.hist(vals, bins=50, color='steelblue', edgecolor='white', linewidth=0.5)
    ax.axvline(vals.mean(), color='red', linestyle='--', label=f'Mean={vals.mean():.3f}')
    ax.set_xlabel('Activation Value')
    ax.set_ylabel('Count')
    ax.set_title(f'Last Conv ({last_name}) Activation Distribution')
    ax.legend()

    # 2. Spatial attention map (mean across all filters)
    ax = axes[1]
    spatial_mean = feat.mean(dim=0).numpy()  # (H, W)
    im = ax.imshow(spatial_mean, origin='lower', cmap='hot')
    ax.set_title('Spatial Attention\n(mean activation across all filters)')
    fig.colorbar(im, ax=ax)

    # 3. Filter importance (mean activation per filter)
    ax = axes[2]
    filter_means = feat.mean(dim=(1, 2)).numpy()  # (C,)
    sorted_idx = np.argsort(filter_means)[::-1]
    ax.bar(range(len(filter_means)), filter_means[sorted_idx], color='steelblue')
    ax.set_xlabel('Filter (sorted by activation)')
    ax.set_ylabel('Mean Activation')
    ax.set_title(f'Filter Importance ({len(filter_means)} filters)')
    # Highlight: how many filters are "dead" (near zero)
    dead = int(np.sum(filter_means < 0.01))
    ax.text(0.7, 0.9, f'Dead filters: {dead}/{len(filter_means)}',
            transform=ax.transAxes, fontsize=10, color='red',
            bbox=dict(boxstyle='round', facecolor='lightyellow'))

    fig.suptitle(f'Last Conv Layer Analysis — Step {step}', fontsize=13, fontweight='bold')
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f'  Saved: {save_path}')
    plt.close(fig)


# ─────────────────────────────────────────────────────────────────────────────
# Main: run episode and capture visualizations
# ─────────────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    import argparse
    from train import make_env_cfg

    parser = argparse.ArgumentParser(description="Visualize CNN input channels")
    parser.add_argument('--model', type=str, default=None, help='Path to a specific model .pt file to evaluate')
    args = parser.parse_args()

    print("=" * 60)
    print("  CNN CHANNEL & FEATURE MAP VISUALIZATION")
    print("=" * 60)

    # ── Setup ──
    out_dir = ROOT / 'outputs' / 'eval' / 'channel_viz'
    os.makedirs(out_dir, exist_ok=True)

    cfg = make_env_cfg()
    env = SwarmSearchPolygonEnv(cfg)
    device = 'cuda' if torch.cuda.is_available() else 'cpu'

    # Find and load model
    def find_model(custom_path=None):
        if custom_path:
            p = Path(custom_path)
            if not p.exists():
                raise FileNotFoundError(f"Provided model path does not exist: {custom_path}")
            return str(p)

        models_dir = ROOT / 'outputs' / 'models'
        for name in ['model_100coverage.pt', 'best_model.pt', 'latest_checkpoint.pt']:
            p = models_dir / name
            if p.exists():
                return str(p)
        pts = sorted(models_dir.glob('*.pt'), key=lambda x: x.stat().st_mtime, reverse=True)
        if pts:
            return str(pts[0])
        raise FileNotFoundError('No model found in outputs/models')

    model_path = find_model(args.model)
    print(f"  Loading model: {model_path}")
    model = ActorCritic(env.actor_obs_spec, env.critic_obs_spec).to(device)
    ckpt = torch.load(model_path, map_location=device)
    sd = ckpt.get('model', ckpt)
    try:
        model.load_state_dict(sd)
    except RuntimeError:
        try:
            model.load_state_dict(sd, strict=False)
        except RuntimeError:
            own = model.state_dict()
            for k, v in sd.items():
                if k in own and own[k].shape == v.shape:
                    own[k] = v
            model.load_state_dict(own)
    model.eval()
    print(f"  Model loaded. Device: {device}")

    # Register hooks for feature map capture
    capture = FeatureMapCapture(model)

    # ── Run episode ──
    obs, info = env.reset(seed=123)

    # Steps to capture detailed visualizations
    # Early, mid, and late game + final steps
    max_steps = env.max_steps
    capture_steps = sorted(set([
        1, 5, 10, 25, 50,                           # early game
        max_steps // 4, max_steps // 2,              # mid game
        int(max_steps * 0.75), int(max_steps * 0.9), # late game
        max_steps - 50, max_steps - 10, max_steps - 1  # final
    ]))
    capture_steps = [s for s in capture_steps if 0 <= s < max_steps]

    # Track channel history for drone 0
    drone_to_track = 0
    channel_history = {}

    # Periodic sampling for evolution plot (every N steps)
    evolution_interval = max(1, max_steps // 30)

    print(f"\n  Running episode ({max_steps} steps)...")
    print(f"  Detailed captures at steps: {capture_steps[:8]}... ({len(capture_steps)} total)")
    print(f"  Evolution sampling every {evolution_interval} steps")
    print()

    for step in range(max_steps):
        obs_t = {k: torch.as_tensor(v, dtype=torch.float32, device=device) for k, v in obs.items()}

        # Capture feature maps at specified steps
        capture.clear()
        with torch.no_grad():
            actions_t, _ = model.actor.act(
                obs_t['maps'], obs_t['self_state'],
                obs_t['neighbor_state'], obs_t['neighbor_mask'],
                deterministic=True)
        actions = actions_t.cpu().numpy()

        # Save channel data for evolution tracking
        maps_np = obs['maps']  # (n_agents, C, H, W)
        self_state_np = obs['self_state']  # (n_agents, D)

        if step % evolution_interval == 0 or step in capture_steps:
            channel_history[step] = maps_np[drone_to_track].copy()

        # Save detailed visualizations at capture steps
        if step in capture_steps:
            coverage_pct = env.last_ever_seen_fraction * 100
            print(f'  Step {step:>4d} | Coverage: {coverage_pct:.1f}% | Capturing...')

            # Input channels for all drones (or just 2 representative)
            drones_to_show = [0, min(2, cfg.max_agents - 1)]
            for d in drones_to_show:
                plot_input_channels(
                    maps_np[d], drone_id=d, step=step,
                    self_state=self_state_np[d],
                    save_path=str(out_dir / f'channels_step{step:04d}_drone{d}.png')
                )

            # Feature maps (for drone 0 only — it was the one processed)
            # The hook captures the full batch; extract drone_to_track
            if capture.features:
                # Feature maps are for the batch (all agents) — slice drone 0
                single_features = {}
                for name, feat in capture.features.items():
                    if feat.shape[0] > drone_to_track:
                        single_features[name] = feat[drone_to_track:drone_to_track+1]

                plot_feature_maps(
                    single_features, drone_id=drone_to_track, step=step,
                    save_path=str(out_dir / f'features_step{step:04d}_drone{drone_to_track}.png')
                )

                plot_last_conv_analysis(
                    single_features, step=step,
                    save_path=str(out_dir / f'last_conv_step{step:04d}.png')
                )

        # Step environment
        obs, reward, terminated, truncated, info = env.step(actions)
        if terminated or truncated:
            print(f'  Episode ended at step {step + 1}')
            break

    # ── Channel evolution plots ──
    print(f'\n  Generating evolution plots ({len(channel_history)} snapshots)...')

    plot_channel_evolution(
        channel_history, drone_id=drone_to_track,
        save_path=str(out_dir / 'channel_evolution.png')
    )

    # Grid: select ~8 representative timesteps for the grid
    grid_steps = sorted(channel_history.keys())
    if len(grid_steps) > 8:
        indices = np.linspace(0, len(grid_steps) - 1, 8, dtype=int)
        grid_steps = [grid_steps[i] for i in indices]
    grid_history = {s: channel_history[s] for s in grid_steps}

    plot_channel_snapshots_grid(
        grid_history, drone_id=drone_to_track,
        save_path=str(out_dir / 'channel_grid.png')
    )

    # ── Summary ──
    capture.remove_hooks()
    final_cov = env.last_ever_seen_fraction * 100
    print(f'\n  Final coverage: {final_cov:.1f}%')
    print(f'  All visualizations saved to: {out_dir}')
    print()
    print("  Files generated:")
    print("    channels_step{N}_drone{D}.png — Raw input channels at specific steps")
    print("    features_step{N}_drone{D}.png — Conv layer feature maps (top-8 active filters)")
    print("    last_conv_step{N}.png         — Last conv layer analysis (histogram + spatial attention)")
    print("    channel_evolution.png         — How channel stats change over time")
    print("    channel_grid.png              — 6 channels × 8 timesteps grid overview")
    print()
    print("  INTERPRETATION GUIDE:")
    print("  ─────────────────────")
    print("  Ch0 (Coverage Freshness): Should start dark, fill with bright as cells get scanned.")
    print("       Problem: If already bright at start → stale cells not properly tracked.")
    print("  Ch1 (Never Visited): Should start bright (all unexplored), darken over time.")
    print("       Problem: If dark patches DON'T match Ch0 bright patches → coverage mismatch.")
    print("  Ch2 (Other Drones): Should show sparse dots. If empty → neighbor info missing.")
    print("  Ch3 (Active Area): Should be constant (polygon shape). If all-1 → no boundary info.")
    print("  Ch4 (Forward FOV): Should rotate with drone heading. If static → heading not updating.")
    print("  Ch5 (Trajectory): Should accumulate over time. If saturated early → decay too slow.")
    print()
    print("  Feature Maps: Look for DEAD filters (all black). Many dead = wasted capacity.")
    print("  Last Conv Spatial Attention: Should focus on unexplored/frontier regions.")
    print("       Problem: If attention is uniform → CNN not learning spatial features.")
    print("  Filter Importance: If >50% filters are dead → network under-utilizing capacity.")

