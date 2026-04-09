# env_swarm_coverage_v0.py
from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Tuple, Any, Optional

import numpy as np
import gymnasium as gym
from gymnasium import spaces
import imageio.v2 as imageio


@dataclass
class SwarmConfig:
    n_agents: int = 4

    # World (2D for simplicity; altitude fixed)
    world_size: float = 20.0   # square area: [-world_size/2, +world_size/2]
    max_speed: float = 3.0     # m/s
    decision_dt: float = 0.2   # RL action update rate (Hz=5)
    internal_dt: float = 0.05  # physics integration step (Hz=20)
    episode_seconds: float = 60.0

    # Coverage grid
    grid_cells: int = 40       # 40x40 grid
    sensor_radius: float = 1.0 # "scan" radius (meters)

    # Formation
    circle_radius: float = 4.0
    desired_spacing: bool = True  # encourage equal angles around centroid

    # Safety
    collision_radius: float = 0.6  # if drones are closer than this -> collision

    # Reward weights
    w_new_coverage: float = 3.0
    w_formation: float = 1.0
    w_spacing: float = 0.5
    w_motion: float = 0.2
    w_boundary: float = 2.0
    w_collision: float = 50.0
    w_smooth: float = 0.05


class SwarmCoverageEnv(gym.Env):
    """
    Multi-drone swarm environment:
    - 4 agents move in a 2D area
    - Each agent chooses a desired position target (x,y)
    - Environment integrates simple kinematics and marks coverage in a grid
    - Reward encourages: new coverage + circle formation + spacing + motion, avoids collisions/bounds
    """

    metadata = {"render_modes": ["rgb_array"], "render_fps": 10}

    def __init__(self, cfg: SwarmConfig | None = None, render_mode: Optional[str] = None):
        super().__init__()
        self.cfg = cfg or SwarmConfig()
        self.render_mode = render_mode

        self.n = self.cfg.n_agents
        assert self.n == 4, "This baseline is written for 4 drones; can be generalized."

        # --- Observation design (per-agent), then stacked for all agents ---
        # obs_i = [
        #   self pos(2), self vel(2),
        #   centroid rel (2),
        #   for each other agent: rel pos(2), rel vel(2)  -> 3*(4)=12
        #   formation error scalar (1),
        #   coverage summary (8): 8-direction local uncovered "gradient"
        # ]
        self.obs_dim_per_agent = 2 + 2 + 2 + 12 + 1 + 8
        obs_shape = (self.n, self.obs_dim_per_agent)

        # --- Action: desired position target (x,y) for each agent ---
        act_shape = (self.n, 2)

        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=obs_shape, dtype=np.float32
        )
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=act_shape, dtype=np.float32
        )
        # action is normalized; we map it to world coordinates.

        # State
        self.pos = np.zeros((self.n, 2), dtype=np.float32)
        self.vel = np.zeros((self.n, 2), dtype=np.float32)
        self.prev_action_world = np.zeros((self.n, 2), dtype=np.float32)

        # Coverage grid: 0 = unvisited, 1 = visited
        self.grid = np.zeros((self.cfg.grid_cells, self.cfg.grid_cells), dtype=np.uint8)

        self.t = 0.0
        self.step_count = 0
        self.max_steps = int(self.cfg.episode_seconds / self.cfg.decision_dt)

        # Rendering cache
        self._last_rgb = None

    # ----------------------------- Utilities -----------------------------

    def _world_to_grid(self, xy: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Map world coords to grid indices."""
        w = self.cfg.world_size
        g = self.cfg.grid_cells
        # world is [-w/2, +w/2]
        u = (xy[:, 0] + w / 2) / w
        v = (xy[:, 1] + w / 2) / w
        ix = np.clip((u * (g - 1)).astype(int), 0, g - 1)
        iy = np.clip((v * (g - 1)).astype(int), 0, g - 1)
        return ix, iy

    def _mark_coverage(self) -> int:
        """Mark scanned cells around each drone; return how many NEW cells were added this step."""
        g = self.cfg.grid_cells
        w = self.cfg.world_size

        before = int(self.grid.sum())

        # mark within sensor radius
        # compute radius in grid units
        cell_size = w / g
        r_cells = max(1, int(self.cfg.sensor_radius / cell_size))

        ix, iy = self._world_to_grid(self.pos)
        for k in range(self.n):
            x0, y0 = ix[k], iy[k]
            xs = slice(max(0, x0 - r_cells), min(g, x0 + r_cells + 1))
            ys = slice(max(0, y0 - r_cells), min(g, y0 + r_cells + 1))
            self.grid[ys, xs] = 1

        after = int(self.grid.sum())
        return after - before

    def _coverage_gradient_8(self, agent_idx: int) -> np.ndarray:
        """
        Simple local coverage cue:
        measure fraction of UNVISITED cells in 8 directions around the agent.
        Returns 8 floats in [0,1].
        """
        g = self.cfg.grid_cells
        w = self.cfg.world_size
        cell_size = w / g

        # sample points in 8 directions
        dirs = np.array([
            [1, 0], [1, 1], [0, 1], [-1, 1],
            [-1, 0], [-1, -1], [0, -1], [1, -1]
        ], dtype=np.float32)
        dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)

        origin = self.pos[agent_idx]
        scores = []
        for d in dirs:
            # look ahead a few meters
            samples = []
            for dist in [1.0, 2.0, 3.0]:
                p = origin + d * dist
                p = np.clip(p, -w / 2, w / 2)
                ix, iy = self._world_to_grid(p.reshape(1, 2))
                samples.append(1.0 - float(self.grid[iy[0], ix[0]]))  # 1 if unvisited
            scores.append(float(np.mean(samples)))
        return np.array(scores, dtype=np.float32)

    def _formation_errors(self) -> Tuple[np.ndarray, float, np.ndarray]:
        """
        Compute:
        - per-agent radial error: |dist_to_centroid - circle_radius|
        - spacing error (optional): how far from equal angles
        """
        c = self.pos.mean(axis=0)
        rel = self.pos - c
        d = np.linalg.norm(rel, axis=1) + 1e-6
        radial_err = np.abs(d - self.cfg.circle_radius)

        spacing_err = 0.0
        spacing_vec = np.zeros(self.n, dtype=np.float32)
        if self.cfg.desired_spacing:
            angles = np.arctan2(rel[:, 1], rel[:, 0])
            angles = (angles + 2 * np.pi) % (2 * np.pi)
            angles_sorted = np.sort(angles)
            diffs = np.diff(np.concatenate([angles_sorted, angles_sorted[:1] + 2 * np.pi]))
            target = 2 * np.pi / self.n
            spacing_err = float(np.mean(np.abs(diffs - target)))

            # map per-agent approx spacing penalty by nearest angle gap mismatch
            # (simple heuristic)
            for i in range(self.n):
                a = angles[i]
                # find nearest in sorted list
                j = int(np.argmin(np.abs(angles_sorted - a)))
                spacing_vec[i] = float(np.abs(diffs[j % self.n] - target))

        return radial_err.astype(np.float32), spacing_err, spacing_vec

    def _collision_cost(self) -> Tuple[float, bool]:
        min_dist = 1e9
        for i in range(self.n):
            for j in range(i + 1, self.n):
                d = float(np.linalg.norm(self.pos[i] - self.pos[j]))
                min_dist = min(min_dist, d)
        collided = min_dist < self.cfg.collision_radius
        # cost increases sharply when close
        if collided:
            return 1.0, True
        # soft cost
        soft = max(0.0, (self.cfg.collision_radius * 2 - min_dist) / (self.cfg.collision_radius * 2))
        return soft, False

    def _boundary_cost(self) -> float:
        w = self.cfg.world_size / 2
        # penalty if near/outside boundary
        margin = 0.5
        cost = 0.0
        for i in range(self.n):
            x, y = self.pos[i]
            dx = max(0.0, abs(float(x)) - (w - margin))
            dy = max(0.0, abs(float(y)) - (w - margin))
            cost += dx + dy
        return float(cost / self.n)

    def _action_to_world(self, a: np.ndarray) -> np.ndarray:
        """
        a in [-1,1] -> desired world target position.
        Map to [-w/2, +w/2].
        """
        w = self.cfg.world_size / 2
        return (a.astype(np.float32) * w)

    # ----------------------------- Gym API -----------------------------

    def reset(self, seed: int | None = None, options: Dict[str, Any] | None = None):
        super().reset(seed=seed)
        rng = np.random.default_rng(seed)

        w = self.cfg.world_size / 2

        # random starts: spread out a bit
        self.pos = rng.uniform(low=-w * 0.6, high=w * 0.6, size=(self.n, 2)).astype(np.float32)
        self.vel = np.zeros((self.n, 2), dtype=np.float32)
        self.prev_action_world = self.pos.copy()

        self.grid[:] = 0
        self._mark_coverage()

        self.t = 0.0
        self.step_count = 0

        obs = self._get_obs()
        info = {"coverage_frac": float(self.grid.mean())}
        return obs, info

    def step(self, action: np.ndarray):
        # clip action
        action = np.clip(action, -1.0, 1.0)
        target_world = self._action_to_world(action)

        # smoothness penalty (large jumps)
        action_jump = np.linalg.norm(target_world - self.prev_action_world, axis=1).mean()
        self.prev_action_world = target_world.copy()

        # integrate internal physics: simple "velocity towards target" with speed limit
        n_sub = int(self.cfg.decision_dt / self.cfg.internal_dt)
        dt = self.cfg.internal_dt

        for _ in range(n_sub):
            direction = target_world - self.pos
            dist = np.linalg.norm(direction, axis=1, keepdims=True) + 1e-6
            desired_vel = direction / dist * self.cfg.max_speed
            # first-order lag for velocity
            tau = 0.3
            self.vel += (desired_vel - self.vel) * (dt / tau)
            speed = np.linalg.norm(self.vel, axis=1, keepdims=True) + 1e-6
            self.vel = self.vel / speed * np.minimum(speed, self.cfg.max_speed)
            self.pos += self.vel * dt

            # keep inside bounds softly (we still penalize boundary)
            w = self.cfg.world_size / 2
            self.pos = np.clip(self.pos, -w, w)

        new_cells = self._mark_coverage()
        coverage_frac = float(self.grid.mean())

        # formation
        radial_err, spacing_err, spacing_vec = self._formation_errors()

        # safety
        collision_soft, collided = self._collision_cost()
        boundary = self._boundary_cost()

        # motion (discourage standing still)
        mean_speed = float(np.linalg.norm(self.vel, axis=1).mean())
        motion_pen = max(0.0, 0.8 - mean_speed)  # penalize if mean speed < 0.8 m/s

        # reward (global cooperative reward)
        r = 0.0
        r += self.cfg.w_new_coverage * (new_cells / (self.cfg.grid_cells * self.cfg.grid_cells))
        r += self.cfg.w_formation * (1.0 - float(radial_err.mean() / (self.cfg.circle_radius + 1e-6)))
        r += self.cfg.w_spacing * (1.0 - float(spacing_err / (np.pi + 1e-6)))
        r -= self.cfg.w_motion * motion_pen
        r -= self.cfg.w_boundary * boundary
        r -= self.cfg.w_smooth * float(action_jump / (self.cfg.world_size + 1e-6))
        r -= self.cfg.w_collision * float(collision_soft)

        terminated = False
        truncated = False

        if collided:
            terminated = True  # episode ends on collision

        self.step_count += 1
        self.t += self.cfg.decision_dt
        if self.step_count >= self.max_steps:
            truncated = True

        obs = self._get_obs()
        info = {
            "coverage_frac": coverage_frac,
            "new_cells": new_cells,
            "mean_speed": mean_speed,
            "radial_err": float(radial_err.mean()),
            "spacing_err": float(spacing_err),
            "collision_soft": float(collision_soft),
            "boundary": float(boundary),
        }
        return obs, float(r), terminated, truncated, info

    def _get_obs(self) -> np.ndarray:
        c = self.pos.mean(axis=0)
        radial_err, _, spacing_vec = self._formation_errors()

        obs = np.zeros((self.n, self.obs_dim_per_agent), dtype=np.float32)

        for i in range(self.n):
            o = []
            o.extend(self.pos[i].tolist())
            o.extend(self.vel[i].tolist())
            o.extend((c - self.pos[i]).tolist())  # centroid relative

            # others relative
            for j in range(self.n):
                if j == i:
                    continue
                o.extend((self.pos[j] - self.pos[i]).tolist())
                o.extend((self.vel[j] - self.vel[i]).tolist())

            o.append(float(radial_err[i] / (self.cfg.circle_radius + 1e-6)))
            o.extend(self._coverage_gradient_8(i).tolist())

            obs[i] = np.array(o, dtype=np.float32)

        return obs

    # ----------------------------- Rendering -----------------------------

    def render(self):
        """
        Simple RGB render of coverage + drone positions.
        Returns RGB array (H,W,3)
        """
        import matplotlib.pyplot as plt
        import io

        g = self.cfg.grid_cells
        fig = plt.figure(figsize=(5, 5))
        ax = fig.add_subplot(111)

        ax.imshow(self.grid, origin="lower", extent=[-1, 1, -1, 1])
        # map positions to [-1,1]
        w = self.cfg.world_size / 2
        p = self.pos / w
        ax.scatter(p[:, 0], p[:, 1])

        ax.set_title("Coverage + Drone positions")
        ax.set_xlim([-1, 1]); ax.set_ylim([-1, 1])

        buf = io.BytesIO()
        fig.canvas.print_png(buf)
        plt.close(fig)

        img = imageio.imread(buf.getvalue())
        self._last_rgb = img
        return img