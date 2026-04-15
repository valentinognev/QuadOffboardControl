from __future__ import annotations

import io
from typing import Dict, List

import imageio.v2 as imageio
import matplotlib.pyplot as plt
import numpy as np


def render_global_frame(state: Dict[str, np.ndarray], world_w: float, world_h: float, title: str) -> np.ndarray:
    fig = plt.figure(figsize=(6, 6))
    ax = fig.add_subplot(111)
    coverage = state['coverage_value']
    ax.imshow(coverage, origin='lower', extent=[-world_w/2, world_w/2, -world_h/2, world_h/2], vmin=0.0, vmax=1.0)
    active = state['active_area']
    ys, xs = np.where(active > 0)
    if len(xs) > 0:
        min_x = (xs.min() / max(1, coverage.shape[1]-1) - 0.5) * world_w
        max_x = (xs.max() / max(1, coverage.shape[1]-1) - 0.5) * world_w
        min_y = (ys.min() / max(1, coverage.shape[0]-1) - 0.5) * world_h
        max_y = (ys.max() / max(1, coverage.shape[0]-1) - 0.5) * world_h
        ax.plot([min_x,max_x,max_x,min_x,min_x],[min_y,min_y,max_y,max_y,min_y],'k--')
    pos = state['positions']
    vel = state['velocities']
    head = state['headings']
    ax.scatter(pos[:,0], pos[:,1], s=60)
    for i in range(pos.shape[0]):
        ax.text(pos[i,0]+0.2, pos[i,1]+0.2, f'D{i}')
        ax.arrow(pos[i,0], pos[i,1], 0.8*np.cos(head[i]), 0.8*np.sin(head[i]), head_width=0.25, length_includes_head=True)
    ax.set_xlim([-world_w/2, world_w/2])
    ax.set_ylim([-world_h/2, world_h/2])
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title(title)
    buf = io.BytesIO()
    fig.canvas.print_png(buf)
    plt.close(fig)
    return imageio.imread(buf.getvalue())


def render_drone_knowledge_frame(state: Dict[str, np.ndarray], drone_idx: int, world_w: float, world_h: float, title: str) -> np.ndarray:
    fig = plt.figure(figsize=(8, 4))
    ax1 = fig.add_subplot(121)
    ax2 = fig.add_subplot(122)

    coverage = state['coverage_value']
    visits = np.clip(np.sum(state['visited_by_mask'][np.arange(state['visited_by_mask'].shape[0]) != drone_idx], axis=0), 0, 1)
    pos = state['positions']
    head = state['headings']

    ax1.imshow(coverage, origin='lower', extent=[-world_w/2, world_w/2, -world_h/2, world_h/2], vmin=0.0, vmax=1.0)
    ax1.scatter(pos[:,0], pos[:,1], s=40)
    ax1.scatter([pos[drone_idx,0]], [pos[drone_idx,1]], s=70, marker='s')
    ax1.set_title(f'{title} - coverage knowledge')
    ax1.set_xlim([-world_w/2, world_w/2])
    ax1.set_ylim([-world_h/2, world_h/2])

    ax2.imshow(visits, origin='lower', extent=[-world_w/2, world_w/2, -world_h/2, world_h/2], vmin=0.0, vmax=1.0)
    ax2.scatter(pos[:,0], pos[:,1], s=40)
    ax2.arrow(pos[drone_idx,0], pos[drone_idx,1], 0.8*np.cos(head[drone_idx]), 0.8*np.sin(head[drone_idx]), head_width=0.25, length_includes_head=True)
    ax2.set_title(f'{title} - others + heading')
    ax2.set_xlim([-world_w/2, world_w/2])
    ax2.set_ylim([-world_h/2, world_h/2])

    buf = io.BytesIO()
    fig.tight_layout()
    fig.canvas.print_png(buf)
    plt.close(fig)
    return imageio.imread(buf.getvalue())


def save_gif(frames: List[np.ndarray], path: str, fps: int = 10):
    imageio.mimsave(path, frames, fps=fps)
