"""Drone state integration and coverage grid."""

from __future__ import annotations

import math

import numpy as np

from config import (
    ALT_M,
    ALPHA_FROM_NADIR_DEG,
    COLOR_ACCUM_MAX_DIST_M,
    DEGRADE_TAU_S,
    FOV_DEG_X,
    FOV_DEG_Y,
    GRID_N,
    INITIAL_T_SIM,
    INITIAL_YAW_RAD,
    VIEWSLOPE,
    initial_pose,
)
from geometry import frustum_ground_quad, mark_footprint
from motion_history import MotionHistory


class DroneSim:
    def __init__(self, history: MotionHistory) -> None:
        self.history = history
        ix, iy = initial_pose()
        self.px = float(ix)
        self.py = float(iy)
        self.yaw = float(INITIAL_YAW_RAD)
        self.t_sim = float(INITIAL_T_SIM)
        self.visited = np.zeros((GRID_N, GRID_N), dtype=bool)
        # Per cell: color value in [0,1]; 0 = red, 1 = blue (FOV ramps up, off-FOV decays).
        self.cell_color_01 = np.zeros((GRID_N, GRID_N), dtype=np.float64)
        self._fov_x = math.radians(FOV_DEG_X)
        self._fov_y = math.radians(FOV_DEG_Y)
        self._alpha = math.radians(ALPHA_FROM_NADIR_DEG)

    def step(self, dt: float) -> None:
        if dt <= 0.0:
            return
        t_prev = self.t_sim
        vx, vy, omega = self.history.sample(self.t_sim)
        self.yaw += omega * dt
        self.px += vx * dt
        self.py += vy * dt
        self.t_sim += dt
        per = self.history.period
        if per is not None and per > 0.0:
            if math.floor(self.t_sim / per) > math.floor(t_prev / per):
                ix, iy = initial_pose()
                self.px = float(ix)
                self.py = float(iy)
                self.yaw = float(INITIAL_YAW_RAD)
        cam = np.array([self.px, self.py], dtype=np.float64)
        tau = max(float(DEGRADE_TAU_S), 1e-9)
        off_decay = math.exp(-dt / tau)
        mark_footprint(
            self.visited,
            cam,
            self.yaw,
            self._fov_x,
            self._fov_y,
            self._alpha,
            cell_color_01=self.cell_color_01,
            color_slope_dt=float(VIEWSLOPE) * dt,
            off_view_decay=off_decay,
            max_accum_dist_m=float(COLOR_ACCUM_MAX_DIST_M),
        )

    def coverage_fraction(self) -> float:
        return float(np.mean(self.visited))

    def mean_color_value_01(self) -> float:
        """Mean per-cell color value in [0,1] (0=red, 1=blue); same scale as accumulation."""
        return float(np.mean(np.clip(self.cell_color_01, 0.0, 1.0)))

    def footprint_quad_xy(self) -> np.ndarray | None:
        """Current camera footprint on ground (4, 2) world XY, CCW; None if degenerate."""
        cam = np.array([self.px, self.py], dtype=np.float64)
        return frustum_ground_quad(
            cam,
            ALT_M,
            self.yaw,
            self._fov_x,
            self._fov_y,
            self._alpha,
        )
