from __future__ import annotations

import io
from typing import Dict, List

import imageio.v2 as imageio
import matplotlib.pyplot as plt
import numpy as np


def _polygon_outline(vertices: np.ndarray):
    closed = np.vstack([vertices, vertices[0:1]])
    return closed[:, 0], closed[:, 1]


def _build_rgb(state: Dict[str, np.ndarray]) -> np.ndarray:
    coverage = state['coverage_value']
    active_area = state['active_area'] > 0
    h, w = coverage.shape
    rgb = np.full((h, w, 3), 0.15, dtype=np.float32)

    cov_inside = coverage * active_area.astype(np.float32)
    rgb[active_area, 1] = np.clip(cov_inside[active_area], 0.12, 1.0)
    rgb[active_area, 0] = np.clip(0.05 + 0.18 * cov_inside[active_area], 0.0, 0.32)
    rgb[active_area, 2] = np.clip(0.04 + 0.10 * cov_inside[active_area], 0.0, 0.20)

    uncovered = state.get('uncovered_mask', None)
    if uncovered is None:
        uncovered = active_area & (state.get('ever_visited', np.ones_like(coverage)) == 0)
    else:
        uncovered = uncovered > 0
    uncovered &= active_area
    rgb[uncovered] = np.array([0.90, 0.08, 0.08], dtype=np.float32)

    stale = state.get('stale_mask', None)
    if stale is not None:
        stale_mask = (stale > 0) & (~uncovered) & active_area
        rgb[stale_mask, 0] = np.clip(rgb[stale_mask, 0] + 0.40, 0, 1)
        rgb[stale_mask, 1] = np.clip(rgb[stale_mask, 1] * 0.65, 0, 1)
        rgb[stale_mask, 2] = np.clip(rgb[stale_mask, 2] * 0.35, 0, 1)
    return rgb


def render_global_frame(
    state: Dict[str, np.ndarray],
    world_w: float,
    world_h: float,
    title: str,
    grid_resolution_m: float | None = None,
) -> np.ndarray:
    _ = grid_resolution_m
    fig, ax = plt.subplots(figsize=(7, 7))
    ext = [-world_w / 2, world_w / 2, -world_h / 2, world_h / 2]
    rgb = _build_rgb(state)
    ax.imshow(rgb, origin='lower', extent=ext, interpolation='nearest')

    if 'polygon_vertices' in state:
        px, py = _polygon_outline(state['polygon_vertices'])
        ax.plot(px, py, 'w-', linewidth=2.0, alpha=0.9)
        ax.plot(px, py, 'k--', linewidth=0.9, alpha=0.6)

    pos = state['positions']
    head = state['headings']
    active_mask = state.get('active_mask', np.ones(pos.shape[0]))
    safety_radius = float(state.get('drone_safety_radius', np.array([4.5], dtype=np.float32))[0])
    colors = plt.cm.tab10(np.linspace(0, 1, pos.shape[0]))

    for i in range(pos.shape[0]):
        if active_mask[i] < 0.5:
            continue
        ax.scatter(pos[i, 0], pos[i, 1], s=80, c=[colors[i]], edgecolors='white', linewidths=1.2, zorder=5)
        ax.annotate(f'D{i}', (pos[i, 0] + 0.35, pos[i, 1] + 0.35), fontsize=8, color='white', fontweight='bold', zorder=6)
        ax.arrow(
            pos[i, 0], pos[i, 1],
            1.2 * np.cos(head[i]), 1.2 * np.sin(head[i]),
            head_width=0.35, length_includes_head=True,
            fc=colors[i], ec='white', linewidth=0.5, zorder=5,
        )
        circle = plt.Circle((pos[i, 0], pos[i, 1]), safety_radius, fill=False, color=colors[i], linestyle=':', linewidth=0.8, alpha=0.5)
        ax.add_patch(circle)

    ax.set_xlim(ext[0], ext[1])
    ax.set_ylim(ext[2], ext[3])
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title(title)
    ax.set_facecolor('#1a1a1a')
    fig.tight_layout()

    buf = io.BytesIO()
    fig.canvas.print_png(buf)
    plt.close(fig)
    return imageio.imread(buf.getvalue())


def render_coverage_summary(
    state: Dict[str, np.ndarray],
    world_w: float,
    world_h: float,
    info: Dict[str, float],
    title: str = 'Coverage Summary',
) -> np.ndarray:
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5), gridspec_kw={'width_ratios': [2, 1]})
    ext = [-world_w / 2, world_w / 2, -world_h / 2, world_h / 2]
    ax1.imshow(_build_rgb(state), origin='lower', extent=ext, interpolation='nearest')
    if 'polygon_vertices' in state:
        px, py = _polygon_outline(state['polygon_vertices'])
        ax1.plot(px, py, 'w-', linewidth=1.5)
    pos = state['positions']
    active_mask = state.get('active_mask', np.ones(pos.shape[0]))
    for i in range(pos.shape[0]):
        if active_mask[i] > 0.5:
            ax1.scatter(pos[i, 0], pos[i, 1], s=45, c='cyan', edgecolors='white', zorder=5)
    ax1.set_title('Coverage Map')
    ax1.set_facecolor('#1a1a1a')

    ax2.axis('off')
    lines = [
        f"Covered at least once: {info.get('percent_covered_at_least_once', 0):.1f}%",
        f"Ever-seen fraction:    {info.get('ever_seen_fraction', 0):.3f}",
        f"Maintained fraction:   {info.get('maintained_fraction', 0):.3f}",
        f"Uncovered fraction:    {info.get('uncovered_fraction', 0):.3f}",
        f"Stale fraction:        {info.get('stale_fraction', 0):.3f}",
        f"Coverage mean:         {info.get('coverage_mean', 0):.3f}",
        f"Time to 50%:           {info.get('time_to_50_coverage', -1):.1f}s",
        f"Time to 80%:           {info.get('time_to_80_coverage', -1):.1f}s",
        f"Time to 95%:           {info.get('time_to_95_coverage', -1):.1f}s",
        f"Min drone dist:        {info.get('min_inter_drone_dist', 0):.2f}m",
        f"Scan efficiency:       {info.get('scan_efficiency_score', 0):.1f}",
    ]
    ax2.text(
        0.05, 0.95, '\n'.join(lines), transform=ax2.transAxes,
        fontsize=11, verticalalignment='top', fontfamily='monospace',
        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.55),
    )
    ax2.set_title(title)

    fig.tight_layout()
    buf = io.BytesIO()
    fig.canvas.print_png(buf)
    plt.close(fig)
    return imageio.imread(buf.getvalue())


def save_gif(frames: List[np.ndarray], path: str, fps: int = 10):
    imageio.mimsave(path, frames, fps=fps)
