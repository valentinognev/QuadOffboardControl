#!/usr/bin/env python3
"""
Run: python main.py (any cwd is fine — script dir is added to sys.path).
- Default trajectory: boustrophedon snake over the map (ROOMBA_TRAJECTORY=snake).
- Constant straight flight: ROOMBA_TRAJECTORY=constant
- Custom: ROOMBA_HISTORY_CSV=/path/to.csv with columns t,vx,vy,yaw_rate (signed yaw vs time)
- Yaw: ROOMBA_YAW_RATE (|omega| rad/s, default pi), ROOMBA_YAW_SIGN (+1/-1 first snake line),
  ROOMBA_CONSTANT_YAW (signed omega for constant trajectory)
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

# Local imports (config, motion_history, …) when cwd is not roombaMovement/ (e.g. VS Code launch).
_SCRIPT_DIR = Path(__file__).resolve().parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

import numpy as np
import pygame

from config import (
    CELL_PIXELS,
    COLOR_ACCUM_MAX_DIST_M,
    DEGRADE_TAU_S,
    FPS,
    GRID_N,
    HISTORY_CSV_PATH,
    PATCH_SIZE_M,
    VIEWSLOPE,
)
from motion_history import load_history
from simulation import DroneSim

# Display RGB endpoints for color value v in [0,1] (0=red, 1=blue)
_RGB_RED = np.array([220.0, 40.0, 40.0], dtype=np.float64)
_RGB_BLUE = np.array([60.0, 120.0, 220.0], dtype=np.float64)


def _color_value_01_to_rgb(v: np.ndarray) -> np.ndarray:
    """Map clipped [0,1] color values to (H,W,3) float RGB."""
    lev = np.clip(v, 0.0, 1.0)
    return lev[..., None] * _RGB_BLUE + (1.0 - lev[..., None]) * _RGB_RED


def _world_to_screen(wx: float, wy: float, grid_px: int) -> tuple[int, int]:
    """Map world XY on the 100×100 m patch to pixel coords (origin top-left, +y down)."""
    sx = int(wx / PATCH_SIZE_M * grid_px)
    sy = int(wy / PATCH_SIZE_M * grid_px)
    return sx, sy


def _draw_overlays(
    target: pygame.Surface,
    sim: DroneSim,
    grid_px: int,
) -> None:
    """Drone position marker and current camera footprint on the grid."""
    quad = sim.footprint_quad_xy()
    if quad is not None:
        pts = [_world_to_screen(float(q[0]), float(q[1]), grid_px) for q in quad]
        ol = pygame.Surface((grid_px, grid_px), pygame.SRCALPHA)
        pygame.draw.polygon(ol, (255, 230, 80, 55), pts)
        target.blit(ol, (0, 0))
        pygame.draw.polygon(target, (255, 200, 0), pts, width=2)

    cx, cy = _world_to_screen(sim.px, sim.py, grid_px)
    r = max(4, CELL_PIXELS)
    pygame.draw.circle(target, (255, 255, 255), (cx, cy), r + 2)
    pygame.draw.circle(target, (30, 30, 220), (cx, cy), r)


def _grid_surface(cell_color_01: np.ndarray) -> pygame.Surface:
    """cell_color_01: per-cell value in [0,1]; RGB is derived only from this scale."""
    h, w = cell_color_01.shape
    mix = _color_value_01_to_rgb(cell_color_01)
    rgb_u8 = np.clip(np.round(mix), 0, 255).astype(np.uint8)
    surf = pygame.surfarray.make_surface(np.ascontiguousarray(rgb_u8))
    return pygame.transform.scale(
        surf, (w * CELL_PIXELS, h * CELL_PIXELS)
    )


def main() -> int:
    history = load_history(HISTORY_CSV_PATH)
    sim = DroneSim(history)

    pygame.init()
    grid_w = GRID_N * CELL_PIXELS
    hud_h = 140
    screen = pygame.display.set_mode((grid_w, grid_w + hud_h))
    pygame.display.set_caption("Roomba-style drone ground scan")
    font = pygame.font.Font(None, 28)
    clock = pygame.time.Clock()

    running = True
    while running:
        dt = clock.tick(FPS) / 1000.0
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                running = False

        sim.step(dt)

        grid_surf = _grid_surface(sim.cell_color_01)
        _draw_overlays(grid_surf, sim, grid_w)
        screen.fill((30, 30, 30))
        screen.blit(grid_surf, (0, 0))

        vx, vy, om = sim.history.sample(sim.t_sim)
        per = sim.history.period
        phase = (sim.t_sim % per) if per is not None and per > 0 else sim.t_sim
        cyc = f"  T_cyc={per:.1f}s phase={phase:.1f}s" if per else ""
        lines = [
            f"t={sim.t_sim:.2f}s  px={sim.px:.1f} py={sim.py:.1f}  yaw={math.degrees(sim.yaw):.1f}deg{cyc}",
            f"vx={vx:.2f} vy={vy:.2f}  yaw_rate={om:.3f} rad/s",
            f"coverage={100.0 * sim.coverage_fraction():.2f}%  slope={VIEWSLOPE:.2f}/s tau={DEGRADE_TAU_S:.1f}s"
            + (
                f" R_acc={COLOR_ACCUM_MAX_DIST_M:.0f}m"
                if math.isfinite(COLOR_ACCUM_MAX_DIST_M)
                else " R_acc=inf"
            ),
            f"mean_color={sim.mean_color_value_01():.3f}  (0=red 1=blue)",
        ]
        y0 = grid_w + 8
        for i, line in enumerate(lines):
            txt = font.render(line, True, (230, 230, 230))
            screen.blit(txt, (8, y0 + i * 30))

        pygame.display.flip()

    pygame.quit()
    return 0


if __name__ == "__main__":
    sys.exit(main())
