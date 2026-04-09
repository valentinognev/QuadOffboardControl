from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

import numpy as np
import gymnasium as gym
from gymnasium import spaces
import imageio.v2 as imageio


@dataclass
class SwarmConfig:
    n_agents: int = 4

    # World
    world_w: float = 30.0
    world_h: float = 30.0
    max_speed: float = 2.5
    decision_dt: float = 0.2
    internal_dt: float = 0.05
    episode_seconds: float = 120.0

    # Coverage map
    grid_w: int = 60
    grid_h: int = 60
    sensor_radius: float = 1.6
    coverage_boost: float = 0.85      # amount added when a drone visits a cell
    decay_per_second: float = 0.030   # map value decays toward 0 with time
    revisit_target: float = 0.35      # cells below this are considered in need of revisit

    # Safety / coordination
    communication_radius: float = 12.0
    soft_neighbor_radius: float = 3.0
    collision_radius: float = 0.60
    boundary_margin: float = 1.0

    # Reward weights (coverage-first; no formation terms)
    w_need_gain: float = 6.0
    w_frontier: float = 1.5
    w_revisit_bonus: float = 1.8
    w_dispersion: float = 0.35
    w_motion: float = 0.45
    w_overlap: float = 2.8
    w_collision: float = 55.0
    w_boundary: float = 2.0
    w_stagnation: float = 0.8
    w_action_smooth: float = 0.03

    # Logging / behavior
    position_history: int = 12


class SwarmCoverageEnv(gym.Env):
    """
    Persistent coverage environment for 4 drones.

    Main design goals:
    - Cover the whole map efficiently.
    - Revisit cells whose coverage value decays over time.
    - Reduce overlap between drones.
    - Encourage exploration and motion.
    - No circle/formation objective.
    """

    metadata = {"render_modes": ["rgb_array"], "render_fps": 10}

    def __init__(self, cfg: SwarmConfig | None = None, render_mode: Optional[str] = None):
        super().__init__()
        self.cfg = cfg or SwarmConfig()
        self.render_mode = render_mode

        self.n = self.cfg.n_agents
        assert self.n == 4, "This version is currently configured for 4 drones."

        # Per-agent observation:
        # self pos(2), self vel(2), wall distances(4), local coverage stats(4),
        # for each other agent: rel pos(2), rel vel(2), rel dist(1) => 3 * 5 = 15,
        # need gradient 8, neighbor density 8, stagnation scalar(1)
        self.obs_dim_per_agent = 2 + 2 + 4 + 4 + 15 + 8 + 8 + 1
        self.observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(self.n, self.obs_dim_per_agent),
            dtype=np.float32,
        )
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(self.n, 2), dtype=np.float32)

        self.pos = np.zeros((self.n, 2), dtype=np.float32)
        self.vel = np.zeros((self.n, 2), dtype=np.float32)
        self.prev_action = np.zeros((self.n, 2), dtype=np.float32)
        self.coverage = np.zeros((self.cfg.grid_h, self.cfg.grid_w), dtype=np.float32)
        self.last_visit_age = np.zeros((self.cfg.grid_h, self.cfg.grid_w), dtype=np.float32)
        self.step_countv = 0
        self.t = 0.0
        self.max_steps = int(self.cfg.episode_seconds / self.cfg.decision_dt)

        self.position_history: list[deque[np.ndarray]] = [
            deque(maxlen=self.cfg.position_history) for _ in range(self.n)
        ]
        self._last_reward_terms: Dict[str, float] = {}
        self._last_rgb = None

    # ------------------------------------------------------------------
    # Geometry helpers
    # ------------------------------------------------------------------
    @property
    def world_half_extents(self) -> Tuple[float, float]:
        return self.cfg.world_w / 2.0, self.cfg.world_h / 2.0

    def _world_to_grid(self, xy: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        hx, hy = self.world_half_extents
        u = (xy[:, 0] + hx) / (2 * hx)
        v = (xy[:, 1] + hy) / (2 * hy)
        ix = np.clip((u * (self.cfg.grid_w - 1)).astype(int), 0, self.cfg.grid_w - 1)
        iy = np.clip((v * (self.cfg.grid_h - 1)).astype(int), 0, self.cfg.grid_h - 1)
        return ix, iy

    def _grid_cell_size(self) -> Tuple[float, float]:
        return self.cfg.world_w / self.cfg.grid_w, self.cfg.world_h / self.cfg.grid_h

    def _cell_centers(self) -> Tuple[np.ndarray, np.ndarray]:
        hx, hy = self.world_half_extents
        xs = np.linspace(-hx, hx, self.cfg.grid_w)
        ys = np.linspace(-hy, hy, self.cfg.grid_h)
        return np.meshgrid(xs, ys)

    def _sensor_mask(self, center_xy: np.ndarray) -> np.ndarray:
        xg, yg = self._cell_centers()
        dist2 = (xg - center_xy[0]) ** 2 + (yg - center_xy[1]) ** 2
        return dist2 <= (self.cfg.sensor_radius ** 2)

    # ------------------------------------------------------------------
    # Coverage and local features
    # ------------------------------------------------------------------
    def _decay_coverage(self) -> None:
        decay = self.cfg.decay_per_second * self.cfg.decision_dt
        self.coverage = np.maximum(0.0, self.coverage - decay)
        self.last_visit_age += self.cfg.decision_dt

    def _mark_coverage(self) -> Tuple[float, float, float]:
        """
        Returns:
            need_gain: weighted gain for covering low-value cells
            revisit_bonus: bonus for revisiting stale cells
            overlap_ratio: average pairwise overlap ratio between agents' scan masks
        """
        masks = [self._sensor_mask(self.pos[i]) for i in range(self.n)]

        need_gain = 0.0
        revisit_bonus = 0.0
        for mask in masks:
            before = self.coverage[mask].copy()
            need_gain += float(np.sum(1.0 - before)) / (self.cfg.grid_w * self.cfg.grid_h)
            revisit_bonus += float(np.sum(before < self.cfg.revisit_target)) / (self.cfg.grid_w * self.cfg.grid_h)
            self.coverage[mask] = np.maximum(self.coverage[mask], self.cfg.coverage_boost)
            self.last_visit_age[mask] = 0.0

        overlap = 0.0
        pairs = 0
        for i in range(self.n):
            for j in range(i + 1, self.n):
                inter = np.logical_and(masks[i], masks[j]).sum()
                union = np.logical_or(masks[i], masks[j]).sum()
                if union > 0:
                    overlap += inter / union
                    pairs += 1
        overlap_ratio = overlap / max(1, pairs)
        return need_gain, revisit_bonus, overlap_ratio

    def _need_gradient_8(self, agent_idx: int, sample_dists=(2.0, 4.0, 6.0)) -> np.ndarray:
        dirs = np.array(
            [[1, 0], [1, 1], [0, 1], [-1, 1], [-1, 0], [-1, -1], [0, -1], [1, -1]],
            dtype=np.float32,
        )
        dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
        origin = self.pos[agent_idx]
        hx, hy = self.world_half_extents
        scores = []
        for d in dirs:
            vals = []
            for dist in sample_dists:
                p = origin + d * dist
                p[0] = np.clip(p[0], -hx, hx)
                p[1] = np.clip(p[1], -hy, hy)
                ix, iy = self._world_to_grid(p.reshape(1, 2))
                vals.append(1.0 - float(self.coverage[iy[0], ix[0]]))
            scores.append(float(np.mean(vals)))
        return np.array(scores, dtype=np.float32)

    def _neighbor_density_8(self, agent_idx: int) -> np.ndarray:
        dirs = np.array(
            [[1, 0], [1, 1], [0, 1], [-1, 1], [-1, 0], [-1, -1], [0, -1], [1, -1]],
            dtype=np.float32,
        )
        dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
        density = np.zeros(8, dtype=np.float32)
        pi = self.pos[agent_idx]
        for j in range(self.n):
            if j == agent_idx:
                continue
            rel = self.pos[j] - pi
            dist = np.linalg.norm(rel) + 1e-6
            reln = rel / dist
            best_dir = int(np.argmax(dirs @ reln))
            density[best_dir] += max(0.0, 1.0 - dist / self.cfg.communication_radius)
        return density

    def _local_coverage_stats(self, agent_idx: int) -> np.ndarray:
        ix, iy = self._world_to_grid(self.pos[agent_idx].reshape(1, 2))
        x, y = int(ix[0]), int(iy[0])
        r = 3
        xs = slice(max(0, x - r), min(self.cfg.grid_w, x + r + 1))
        ys = slice(max(0, y - r), min(self.cfg.grid_h, y + r + 1))
        patch = self.coverage[ys, xs]
        return np.array(
            [
                float(np.mean(patch)),
                float(np.min(patch)),
                float(np.max(patch)),
                float(np.mean(patch < self.cfg.revisit_target)),
            ],
            dtype=np.float32,
        )

    # ------------------------------------------------------------------
    # Costs and rewards
    # ------------------------------------------------------------------
    def _collision_cost(self) -> Tuple[float, bool, float]:
        min_dist = 1e9
        soft = 0.0
        for i in range(self.n):
            for j in range(i + 1, self.n):
                d = float(np.linalg.norm(self.pos[i] - self.pos[j]))
                min_dist = min(min_dist, d)
                if d < self.cfg.soft_neighbor_radius:
                    soft += (self.cfg.soft_neighbor_radius - d) / self.cfg.soft_neighbor_radius
        collided = min_dist < self.cfg.collision_radius
        soft /= max(1, self.n * (self.n - 1) / 2)
        return soft, collided, float(min_dist)

    def _boundary_cost(self) -> float:
        hx, hy = self.world_half_extents
        margin = self.cfg.boundary_margin
        cost = 0.0
        for x, y in self.pos:
            dx = max(0.0, abs(float(x)) - (hx - margin))
            dy = max(0.0, abs(float(y)) - (hy - margin))
            cost += dx + dy
        return cost / self.n

    def _dispersion_reward(self) -> float:
        vals = []
        for i in range(self.n):
            for j in range(i + 1, self.n):
                d = float(np.linalg.norm(self.pos[i] - self.pos[j]))
                vals.append(min(d / self.cfg.communication_radius, 1.0))
        return float(np.mean(vals)) if vals else 0.0

    def _stagnation_penalty(self) -> float:
        penalties = []
        for hist in self.position_history:
            if len(hist) < hist.maxlen:
                penalties.append(0.0)
                continue
            traveled = 0.0
            hist_list = list(hist)
            for a, b in zip(hist_list[:-1], hist_list[1:]):
                traveled += float(np.linalg.norm(b - a))
            penalties.append(max(0.0, 1.2 - traveled) / 1.2)
        return float(np.mean(penalties)) if penalties else 0.0

    def _frontier_bonus(self) -> float:
        low_mask = self.coverage < self.cfg.revisit_target
        if not np.any(low_mask):
            return 0.0
        ys, xs = np.where(low_mask)
        cell_x = (xs / max(1, self.cfg.grid_w - 1) - 0.5) * self.cfg.world_w
        cell_y = (ys / max(1, self.cfg.grid_h - 1) - 0.5) * self.cfg.world_h
        points = np.stack([cell_x, cell_y], axis=1)
        dists = []
        for i in range(self.n):
            d = np.linalg.norm(points - self.pos[i], axis=1)
            dists.append(float(np.min(d)))
        # bonus when at least some agent is closing the gap to underserved area
        return float(np.exp(-np.mean(dists) / 8.0))

    def _action_to_velocity_target(self, action: np.ndarray) -> np.ndarray:
        return action.astype(np.float32) * self.cfg.max_speed

    # ------------------------------------------------------------------
    # Gym API
    # ------------------------------------------------------------------
    def reset(self, seed: int | None = None, options: Dict[str, Any] | None = None):
        super().reset(seed=seed)
        rng = np.random.default_rng(seed)
        hx, hy = self.world_half_extents

        # Start spread out across the map for better exploration.
        base = np.array([
            [-0.45 * hx, -0.45 * hy],
            [0.45 * hx, -0.45 * hy],
            [-0.45 * hx, 0.45 * hy],
            [0.45 * hx, 0.45 * hy],
        ], dtype=np.float32)
        noise = rng.normal(0.0, 0.7, size=(self.n, 2)).astype(np.float32)
        self.pos = np.clip(base + noise, [-hx, -hy], [hx, hy]).astype(np.float32)
        self.vel = np.zeros((self.n, 2), dtype=np.float32)
        self.prev_action = np.zeros((self.n, 2), dtype=np.float32)
        self.coverage.fill(0.0)
        self.last_visit_age.fill(1e3)
        self._decay_coverage()  # no-op for coverage, but keeps behavior consistent
        self._mark_coverage()
        self.step_count = 0
        self.t = 0.0
        for h in self.position_history:
            h.clear()
        for i in range(self.n):
            self.position_history[i].append(self.pos[i].copy())

        obs = self._get_obs()
        info = self._build_info(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, False, 0.0)
        return obs, info

    def step(self, action: np.ndarray):
        action = np.clip(action, -1.0, 1.0)
        velocity_target = self._action_to_velocity_target(action)
        action_jump = float(np.linalg.norm(velocity_target - self.prev_action, axis=1).mean())
        self.prev_action = velocity_target.copy()

        self._decay_coverage()

        n_sub = int(self.cfg.decision_dt / self.cfg.internal_dt)
        dt = self.cfg.internal_dt
        hx, hy = self.world_half_extents
        for _ in range(n_sub):
            tau = 0.35
            self.vel += (velocity_target - self.vel) * (dt / tau)
            speed = np.linalg.norm(self.vel, axis=1, keepdims=True) + 1e-6
            self.vel = self.vel / speed * np.minimum(speed, self.cfg.max_speed)
            self.pos += self.vel * dt
            self.pos[:, 0] = np.clip(self.pos[:, 0], -hx, hx)
            self.pos[:, 1] = np.clip(self.pos[:, 1], -hy, hy)

        for i in range(self.n):
            self.position_history[i].append(self.pos[i].copy())

        need_gain, revisit_bonus, overlap_ratio = self._mark_coverage()
        collision_soft, collided, min_dist = self._collision_cost()
        boundary_cost = self._boundary_cost()
        dispersion = self._dispersion_reward()
        stagnation = self._stagnation_penalty()
        frontier_bonus = self._frontier_bonus()
        mean_speed = float(np.linalg.norm(self.vel, axis=1).mean())
        motion_penalty = max(0.0, 0.9 - mean_speed)

        reward_terms = {
            "need_gain": self.cfg.w_need_gain * need_gain,
            "frontier_bonus": self.cfg.w_frontier * frontier_bonus,
            "revisit_bonus": self.cfg.w_revisit_bonus * revisit_bonus,
            "dispersion_reward": self.cfg.w_dispersion * dispersion,
            "motion_penalty": -self.cfg.w_motion * motion_penalty,
            "overlap_penalty": -self.cfg.w_overlap * overlap_ratio,
            "collision_penalty": -self.cfg.w_collision * collision_soft,
            "boundary_penalty": -self.cfg.w_boundary * boundary_cost,
            "stagnation_penalty": -self.cfg.w_stagnation * stagnation,
            "action_smooth_penalty": -self.cfg.w_action_smooth * (action_jump / max(1e-6, self.cfg.max_speed)),
        }
        reward = float(sum(reward_terms.values()))
        self._last_reward_terms = reward_terms

        terminated = bool(collided)
        self.step_count += 1
        self.t += self.cfg.decision_dt
        truncated = self.step_count >= self.max_steps

        obs = self._get_obs()
        info = self._build_info(
            reward,
            need_gain,
            revisit_bonus,
            overlap_ratio,
            frontier_bonus,
            boundary_cost,
            collision_soft,
            stagnation,
            motion_penalty,
            terminated,
            min_dist,
        )
        return obs, reward, terminated, truncated, info

    def _build_info(
        self,
        reward: float,
        need_gain: float,
        revisit_bonus: float,
        overlap_ratio: float,
        frontier_bonus: float,
        boundary_cost: float,
        collision_soft: float,
        stagnation: float,
        motion_penalty: float,
        collided: bool,
        min_dist: float,
    ) -> Dict[str, float]:
        return {
            "reward_total": float(reward),
            "coverage_mean": float(np.mean(self.coverage)),
            "coverage_min": float(np.min(self.coverage)),
            "coverage_max": float(np.max(self.coverage)),
            "underserved_fraction": float(np.mean(self.coverage < self.cfg.revisit_target)),
            "need_gain": float(need_gain),
            "revisit_bonus": float(revisit_bonus),
            "frontier_bonus": float(frontier_bonus),
            "overlap_ratio": float(overlap_ratio),
            "boundary_cost": float(boundary_cost),
            "collision_soft": float(collision_soft),
            "collided": float(collided),
            "stagnation": float(stagnation),
            "motion_penalty": float(motion_penalty),
            "mean_speed": float(np.linalg.norm(self.vel, axis=1).mean()),
            "min_inter_drone_dist": float(min_dist),
            **{f"term_{k}": float(v) for k, v in self._last_reward_terms.items()},
        }

    def _get_obs(self) -> np.ndarray:
        hx, hy = self.world_half_extents
        obs = np.zeros((self.n, self.obs_dim_per_agent), dtype=np.float32)

        for i in range(self.n):
            x, y = self.pos[i]
            vx, vy = self.vel[i]
            wall_dists = np.array(
                [
                    (hx - x) / self.cfg.world_w,
                    (x + hx) / self.cfg.world_w,
                    (hy - y) / self.cfg.world_h,
                    (y + hy) / self.cfg.world_h,
                ],
                dtype=np.float32,
            )
            local_cov = self._local_coverage_stats(i)
            need_grad = self._need_gradient_8(i)
            neigh_dens = self._neighbor_density_8(i)

            feat = [x / hx, y / hy, vx / self.cfg.max_speed, vy / self.cfg.max_speed]
            feat.extend(wall_dists.tolist())
            feat.extend(local_cov.tolist())
            for j in range(self.n):
                if j == i:
                    continue
                rel = self.pos[j] - self.pos[i]
                rel_v = self.vel[j] - self.vel[i]
                dist = np.linalg.norm(rel)
                feat.extend(
                    [
                        rel[0] / self.cfg.world_w,
                        rel[1] / self.cfg.world_h,
                        rel_v[0] / self.cfg.max_speed,
                        rel_v[1] / self.cfg.max_speed,
                        min(dist / self.cfg.communication_radius, 1.5),
                    ]
                )
            feat.extend(need_grad.tolist())
            feat.extend(neigh_dens.tolist())

            hist = self.position_history[i]
            if len(hist) >= 2:
                traveled = float(sum(np.linalg.norm(b - a) for a, b in zip(list(hist)[:-1], list(hist)[1:])))
                stagnation_scalar = max(0.0, 1.2 - traveled) / 1.2
            else:
                stagnation_scalar = 0.0
            feat.append(stagnation_scalar)

            obs[i] = np.array(feat, dtype=np.float32)
        return obs

    # ------------------------------------------------------------------
    # Rendering
    # ------------------------------------------------------------------
    def render(self):
        import io
        import matplotlib.pyplot as plt

        fig = plt.figure(figsize=(6, 6))
        ax = fig.add_subplot(111)
        hx, hy = self.world_half_extents
        im = ax.imshow(
            self.coverage,
            origin="lower",
            extent=[-hx, hx, -hy, hy],
            vmin=0.0,
            vmax=1.0,
        )
        fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04, label="coverage value")
        ax.scatter(self.pos[:, 0], self.pos[:, 1], s=70)
        for i in range(self.n):
            ax.text(self.pos[i, 0] + 0.2, self.pos[i, 1] + 0.2, f"D{i}")
            circ = plt.Circle((self.pos[i, 0], self.pos[i, 1]), self.cfg.sensor_radius, fill=False, linestyle="--", alpha=0.4)
            ax.add_patch(circ)
        ax.set_title("Persistent coverage map + drone positions")
        ax.set_xlim([-hx, hx])
        ax.set_ylim([-hy, hy])
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        buf = io.BytesIO()
        fig.canvas.print_png(buf)
        plt.close(fig)
        img = imageio.imread(buf.getvalue())
        self._last_rgb = img
        return img
