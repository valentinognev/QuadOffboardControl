from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np


@dataclass
class EnvConfig:
    n_agents: int = 4
    world_w: float = 20.0
    world_h: float = 20.0
    decision_hz: float = 10.0
    episode_seconds: float = 90.0
    max_speed: float = 2.0
    max_turn_rate_deg: float = 180.0

    drone_safety_radius: float = 0.50
    wall_safety_margin: float = 0.05
    sensor_forward_range: float = 0.40
    heading_fov_deg: float = 80.0
    full_scan_radius: float = 1.25

    decay_reset_seconds: float = 60.0
    grid_w: int = 80
    grid_h: int = 80
    local_map_size: int = 31
    start_circle_radius_min: float = 2.0
    start_circle_radius_max: float = 7.0
    start_center_margin: float = 0.5

    w_first_visit: float = 40.0
    w_new_area_delta: float = 35.0
    w_ever_seen_gain: float = 70.0
    w_coverage_gain: float = 5.0
    w_frontier_progress: float = 4.0 # reward for getting closer to the frontier, to encourage exploring never -covered areas
    w_dispersion: float = 1.2
    w_overlap: float = 10.0
    w_crowding: float = 10.0
    w_collision: float = 70.0
    w_wall: float = 50.0
    w_stagnation: float = 30.0
    w_time: float = 0.03


class SwarmCoverageEnv:
    def __init__(self, cfg: EnvConfig | None = None):
        self.cfg = cfg or EnvConfig()
        self.n_agents = self.cfg.n_agents
        self.dt = 1.0 / self.cfg.decision_hz
        self.max_steps = int(self.cfg.episode_seconds * self.cfg.decision_hz)

        self.pos = np.zeros((self.n_agents, 2), dtype=np.float32)
        self.vel = np.zeros((self.n_agents, 2), dtype=np.float32)
        self.heading = np.zeros(self.n_agents, dtype=np.float32)
        self.prev_pos = np.zeros((self.n_agents, 2), dtype=np.float32)

        self.coverage_age = np.full((self.cfg.grid_h, self.cfg.grid_w), self.cfg.decay_reset_seconds, dtype=np.float32)
        self.coverage_value = np.zeros((self.cfg.grid_h, self.cfg.grid_w), dtype=np.float32)
        self.visited_by_mask = np.zeros((self.n_agents, self.cfg.grid_h, self.cfg.grid_w), dtype=np.uint8)
        self.ever_visited = np.zeros((self.cfg.grid_h, self.cfg.grid_w), dtype=np.uint8)
        self.last_owner = -np.ones((self.cfg.grid_h, self.cfg.grid_w), dtype=np.int16)

        self.step_count = 0
        self.last_ever_seen_fraction = 0.0
        self.time_to_50 = -1.0
        self.time_to_80 = -1.0
        self.time_to_95 = -1.0

    @property
    def observation_channels(self) -> int:
        return 7

    @property
    def self_state_dim(self) -> int:
        return 15

    @property
    def actor_obs_spec(self) -> Tuple[Tuple[int, int, int], int]:
        return (self.observation_channels, self.cfg.local_map_size, self.cfg.local_map_size), self.self_state_dim

    @property
    def centralized_state_dim(self) -> int:
        # 4 maps @ 4x4 + all agents 7 values + extras
        return 4 * 4 * 4 + self.n_agents * 7 + 7

    def get_active_bounds(self) -> Tuple[float, float, float, float]:
        hx = self.cfg.world_w / 2.0
        hy = self.cfg.world_h / 2.0
        return -hx, hx, -hy, hy

    def reset(self, seed: int | None = None):
        _ = np.random.default_rng(seed)
        self.step_count = 0
        self.coverage_age.fill(self.cfg.decay_reset_seconds)
        self.coverage_value.fill(0.0)
        self.visited_by_mask.fill(0)
        self.ever_visited.fill(0)
        self.last_owner.fill(-1)
        self.time_to_50 = -1.0
        self.time_to_80 = -1.0
        self.time_to_95 = -1.0
        self.last_ever_seen_fraction = 0.0

        # Random circle center in the mission area + random circle radius.
        base_angles = np.linspace(0.0, 2.0 * math.pi, self.n_agents, endpoint=False)
        angular_offset = float(_.uniform(0.0, 2.0 * math.pi))
        angles = base_angles + angular_offset
        r = float(_.uniform(self.cfg.start_circle_radius_min, self.cfg.start_circle_radius_max))

        # Keep the full circle safely inside the sector.
        hx = self.cfg.world_w / 2.0
        hy = self.cfg.world_h / 2.0
        safe_margin = max(self.cfg.wall_safety_margin + 0.25, self.cfg.start_center_margin)
        cx = float(_.uniform(-hx + r + safe_margin, hx - r - safe_margin))
        cy = float(_.uniform(-hy + r + safe_margin, hy - r - safe_margin))
        center = np.array([cx, cy], dtype=np.float32)

        self.pos[:, 0] = center[0] + r * np.cos(angles)
        self.pos[:, 1] = center[1] + r * np.sin(angles)
        self.vel.fill(0.0)
        # Initial headings point outward from the random center.
        self.heading = angles.astype(np.float32)
        self.prev_pos = self.pos.copy()

        self._update_coverage_and_visits()
        obs = self._build_actor_obs()
        info = self._build_info(np.zeros(self.n_agents, dtype=np.float32))
        return obs, info

    def step(self, actions: np.ndarray):
        actions = np.asarray(actions, dtype=np.float32)
        assert actions.shape == (self.n_agents, 3)

        self.prev_pos = self.pos.copy()
        speed_cmd = np.clip((actions[:, 0] + 1.0) * 0.5, 0.0, 1.0) * self.cfg.max_speed
        turn_cmd = np.clip(actions[:, 1], -1.0, 1.0) * math.radians(self.cfg.max_turn_rate_deg)
        lateral_cmd = np.clip(actions[:, 2], -1.0, 1.0)

        self.heading += turn_cmd * self.dt
        self.heading = ((self.heading + math.pi) % (2.0 * math.pi)) - math.pi

        forward = np.stack([np.cos(self.heading), np.sin(self.heading)], axis=1)
        lateral = np.stack([-np.sin(self.heading), np.cos(self.heading)], axis=1)
        self.vel = forward * speed_cmd[:, None] + lateral * (0.35 * self.cfg.max_speed * lateral_cmd[:, None])
        self.pos += self.vel * self.dt

        min_x, max_x, min_y, max_y = self.get_active_bounds()
        self.pos[:, 0] = np.clip(self.pos[:, 0], min_x, max_x)
        self.pos[:, 1] = np.clip(self.pos[:, 1], min_y, max_y)

        self.step_count += 1

        cov_stats = self._update_coverage_and_visits()
        overlap_ratio = self._overlap_ratio()
        crowding_penalty = self._crowding_penalty()
        collision_penalty, min_pair_dist = self._collision_stats()
        wall_penalty = self._wall_violation_stats()
        stagnation_penalty = self._stagnation_penalty()
        frontier_progress = self._progress_to_frontier_bonus()
        dispersion_reward = self._dispersion_reward()

        rewards = np.zeros(self.n_agents, dtype=np.float32)
        rewards += self.cfg.w_first_visit * cov_stats['first_visit_gain']
        rewards += self.cfg.w_new_area_delta * cov_stats['new_area_delta']
        rewards += self.cfg.w_ever_seen_gain * cov_stats['ever_seen_gain']
        rewards += self.cfg.w_coverage_gain * cov_stats['coverage_gain']
        rewards += self.cfg.w_frontier_progress * frontier_progress
        rewards += self.cfg.w_dispersion * dispersion_reward
        rewards -= self.cfg.w_overlap * overlap_ratio
        rewards -= self.cfg.w_crowding * crowding_penalty
        rewards -= self.cfg.w_collision * collision_penalty
        rewards -= self.cfg.w_wall * wall_penalty
        rewards -= self.cfg.w_stagnation * stagnation_penalty
        rewards -= self.cfg.w_time

        terminated = False
        truncated = self.step_count >= self.max_steps
        obs = self._build_actor_obs()
        info = self._build_info(rewards)
        info.update({
            'overlap_ratio_mean': float(np.mean(overlap_ratio)),
            'crowding_penalty_mean': float(np.mean(crowding_penalty)),
            'collision_penalty_mean': float(np.mean(collision_penalty)),
            'wall_penalty_mean': float(np.mean(wall_penalty)),
            'stagnation_mean': float(np.mean(stagnation_penalty)),
            'frontier_progress_mean': float(np.mean(frontier_progress)),
            'dispersion_reward_mean': float(np.mean(dispersion_reward)),
            'min_inter_drone_dist': float(min_pair_dist),
        })
        return obs, rewards, terminated, truncated, info

    def _world_to_grid(self, pts: np.ndarray):
        x = ((pts[:, 0] + self.cfg.world_w / 2.0) / self.cfg.world_w) * (self.cfg.grid_w - 1)
        y = ((pts[:, 1] + self.cfg.world_h / 2.0) / self.cfg.world_h) * (self.cfg.grid_h - 1)
        ix = np.clip(np.round(x).astype(int), 0, self.cfg.grid_w - 1)
        iy = np.clip(np.round(y).astype(int), 0, self.cfg.grid_h - 1)
        return ix, iy

    def _grid_to_world(self, ix: np.ndarray, iy: np.ndarray):
        x = (ix / max(1, self.cfg.grid_w - 1) - 0.5) * self.cfg.world_w
        y = (iy / max(1, self.cfg.grid_h - 1) - 0.5) * self.cfg.world_h
        return np.stack([x, y], axis=-1)

    def _active_area_mask(self) -> np.ndarray:
        return np.ones((self.cfg.grid_h, self.cfg.grid_w), dtype=bool)

    def _scan_mask_for_agent(self, agent_idx: int) -> np.ndarray:
        gx, gy = np.meshgrid(np.arange(self.cfg.grid_w), np.arange(self.cfg.grid_h))
        pts = self._grid_to_world(gx, gy)
        rel = pts - self.pos[agent_idx][None, None, :]
        dist = np.linalg.norm(rel, axis=-1)
        angle = np.arctan2(rel[..., 1], rel[..., 0]) - self.heading[agent_idx]
        angle = (angle + np.pi) % (2.0 * np.pi) - np.pi
        half_fov = math.radians(self.cfg.heading_fov_deg) / 2.0
        return (dist <= self.cfg.full_scan_radius) & (np.abs(angle) <= half_fov)

    def _update_coverage_and_visits(self) -> Dict[str, np.ndarray]:
        prev_ever = float(np.mean(self.ever_visited[self._active_area_mask()]))
        self.coverage_age = np.minimum(self.coverage_age + self.dt, self.cfg.decay_reset_seconds)
        self.coverage_value = np.clip(1.0 - self.coverage_age / self.cfg.decay_reset_seconds, 0.0, 1.0)

        first_visit_gain = np.zeros(self.n_agents, dtype=np.float32)
        coverage_gain = np.zeros(self.n_agents, dtype=np.float32)
        new_area_delta = np.zeros(self.n_agents, dtype=np.float32)

        ownership_masks = [self._scan_mask_for_agent(i) for i in range(self.n_agents)]
        combined = np.zeros_like(self.coverage_value, dtype=np.int16) - 1
        if ownership_masks:
            visible_any = np.any(np.stack(ownership_masks, axis=0), axis=0)
            if np.any(visible_any):
                visible_coords = np.argwhere(visible_any)
                world_pts = self._grid_to_world(visible_coords[:, 1], visible_coords[:, 0])
                d_all = np.linalg.norm(world_pts[:, None, :] - self.pos[None, :, :], axis=-1)
                owner_ids = np.argmin(d_all, axis=1)
                combined[visible_coords[:, 0], visible_coords[:, 1]] = owner_ids.astype(np.int16)

        total_active = self.cfg.grid_w * self.cfg.grid_h
        for i in range(self.n_agents):
            owned = (combined == i)
            if not np.any(owned):
                continue
            prev_vis = self.ever_visited[owned].copy()
            prev_cov = self.coverage_value[owned].copy()
            first_visit_gain[i] = float(np.sum(prev_vis == 0)) / total_active
            coverage_gain[i] = float(np.sum(1.0 - prev_cov)) / total_active
            new_area_delta[i] = first_visit_gain[i]

            self.coverage_age[owned] = 0.0
            self.coverage_value[owned] = 1.0
            self.ever_visited[owned] = 1
            self.visited_by_mask[i, owned] = 1
            self.last_owner[owned] = i

        current_ever = float(np.mean(self.ever_visited))
        ever_gain_scalar = max(0.0, current_ever - prev_ever)
        ever_seen_gain = np.full(self.n_agents, ever_gain_scalar, dtype=np.float32)
        self.last_ever_seen_fraction = current_ever

        current_time_s = self.step_count * self.dt
        if self.time_to_50 < 0 and current_ever >= 0.50:
            self.time_to_50 = current_time_s
        if self.time_to_80 < 0 and current_ever >= 0.80:
            self.time_to_80 = current_time_s
        if self.time_to_95 < 0 and current_ever >= 0.95:
            self.time_to_95 = current_time_s

        return {
            'first_visit_gain': first_visit_gain,
            'coverage_gain': coverage_gain,
            'new_area_delta': new_area_delta,
            'ever_seen_gain': ever_seen_gain,
        }

    def _overlap_ratio(self) -> np.ndarray:
        masks = [self._scan_mask_for_agent(i) for i in range(self.n_agents)]
        ratios = np.zeros(self.n_agents, dtype=np.float32)
        for i in range(self.n_agents):
            vals = []
            for j in range(self.n_agents):
                if i == j:
                    continue
                inter = np.logical_and(masks[i], masks[j]).sum()
                union = np.logical_or(masks[i], masks[j]).sum()
                vals.append(inter / max(1, union))
            ratios[i] = float(np.mean(vals)) if vals else 0.0
        return ratios

    def _crowding_penalty(self) -> np.ndarray:
        penalties = np.zeros(self.n_agents, dtype=np.float32)
        for i in range(self.n_agents):
            vals = []
            for j in range(self.n_agents):
                if i == j:
                    continue
                d = float(np.linalg.norm(self.pos[i] - self.pos[j]))
                vals.append(max(0.0, 1.0 - d / 3.0))
            penalties[i] = float(np.mean(vals)) if vals else 0.0
        return penalties

    def _dispersion_reward(self) -> np.ndarray:
        rewards = np.zeros(self.n_agents, dtype=np.float32)
        for i in range(self.n_agents):
            vals = []
            for j in range(self.n_agents):
                if i == j:
                    continue
                d = float(np.linalg.norm(self.pos[i] - self.pos[j]))
                vals.append(min(d / 6.0, 1.0))
            rewards[i] = float(np.mean(vals)) if vals else 0.0
        return rewards

    def _collision_stats(self):
        penalties = np.zeros(self.n_agents, dtype=np.float32)
        min_dist = 1e9
        for i in range(self.n_agents):
            for j in range(i + 1, self.n_agents):
                d = float(np.linalg.norm(self.pos[i] - self.pos[j]))
                min_dist = min(min_dist, d)
                if d < self.cfg.drone_safety_radius:
                    penalties[i] += 1.0
                    penalties[j] += 1.0
                elif d < 2.0 * self.cfg.drone_safety_radius:
                    p = (2.0 * self.cfg.drone_safety_radius - d) / max(1e-6, self.cfg.drone_safety_radius)
                    penalties[i] += p
                    penalties[j] += p
        penalties /= max(1, self.n_agents - 1)
        return penalties, min_dist

    def _wall_violation_stats(self):
        penalties = np.zeros(self.n_agents, dtype=np.float32)
        min_x, max_x, min_y, max_y = self.get_active_bounds()
        for i in range(self.n_agents):
            x, y = self.pos[i]
            dists = np.array([max_x - x, x - min_x, max_y - y, y - min_y], dtype=np.float32)
            min_wall = float(np.min(dists))
            if min_wall < self.cfg.wall_safety_margin:
                penalties[i] = 1.0 + (self.cfg.wall_safety_margin - min_wall)
            elif min_wall < 0.5:
                penalties[i] = (0.5 - min_wall) / 0.5
        return penalties

    def _stagnation_penalty(self):
        moved = np.linalg.norm(self.pos - self.prev_pos, axis=1)
        return np.clip((0.05 - moved) / 0.05, 0.0, 1.0).astype(np.float32)

    def _progress_to_frontier_bonus(self):
        frontier_mask = self.ever_visited == 0
        bonus = np.zeros(self.n_agents, dtype=np.float32)
        if not np.any(frontier_mask):
            return bonus
        ys, xs = np.where(frontier_mask)
        sample_step = max(1, len(xs) // 400)
        xs = xs[::sample_step]
        ys = ys[::sample_step]
        pts = self._grid_to_world(xs, ys)
        for i in range(self.n_agents):
            now = np.min(np.linalg.norm(pts - self.pos[i], axis=1))
            prev = np.min(np.linalg.norm(pts - self.prev_pos[i], axis=1))
            bonus[i] = max(0.0, prev - now)
        return bonus.astype(np.float32)

    def _crop_channel(self, channel: np.ndarray, center_ix: int, center_iy: int) -> np.ndarray:
        size = self.cfg.local_map_size
        r = size // 2
        padded = np.pad(channel, ((r, r), (r, r)), mode='constant')
        crop = padded[center_iy:center_iy + size, center_ix:center_ix + size]
        return crop.astype(np.float32)

    def _forward_sensor_map(self) -> np.ndarray:
        local = np.zeros((self.cfg.local_map_size, self.cfg.local_map_size), dtype=np.float32)
        size = self.cfg.local_map_size
        center = size // 2
        radius_cells = max(1, int(round(self.cfg.sensor_forward_range / (self.cfg.world_w / self.cfg.grid_w))))
        half_width = max(1, int(radius_cells * 0.8))
        for dy in range(-half_width, half_width + 1):
            for dx in range(0, radius_cells + 1):
                x = center + dx
                y = center + dy
                if 0 <= x < size and 0 <= y < size:
                    local[y, x] = 1.0
        return local

    def _build_actor_obs(self) -> List[Dict[str, np.ndarray]]:
        ix, iy = self._world_to_grid(self.pos)
        local_obs: List[Dict[str, np.ndarray]] = []
        active_area = self._active_area_mask().astype(np.float32)
        sensor_crop = self._forward_sensor_map()
        for i in range(self.n_agents):
            visited_by_others = np.clip(np.sum(self.visited_by_mask[np.arange(self.n_agents) != i], axis=0), 0, 1).astype(np.float32)
            other_drones = np.zeros((self.cfg.grid_h, self.cfg.grid_w), dtype=np.float32)
            for j in range(self.n_agents):
                if j != i:
                    other_drones[iy[j], ix[j]] = 1.0
            self_map = np.zeros((self.cfg.grid_h, self.cfg.grid_w), dtype=np.float32)
            self_map[iy[i], ix[i]] = 1.0
            never_visited = 1.0 - self.ever_visited.astype(np.float32)

            coverage = self._crop_channel(self.coverage_value, ix[i], iy[i])
            others_visit = self._crop_channel(visited_by_others, ix[i], iy[i])
            drones_map = self._crop_channel(other_drones, ix[i], iy[i])
            self_crop = self._crop_channel(self_map, ix[i], iy[i])
            never_crop = self._crop_channel(never_visited, ix[i], iy[i])
            active_crop = self._crop_channel(active_area, ix[i], iy[i])
            maps = np.stack([coverage, others_visit, drones_map, sensor_crop, never_crop, self_crop, active_crop], axis=0).astype(np.float32)

            hx = self.cfg.world_w / 2.0
            hy = self.cfg.world_h / 2.0
            wall_dists = np.array([
                (hx - self.pos[i, 0]) / self.cfg.world_w,
                (self.pos[i, 0] + hx) / self.cfg.world_w,
                (hy - self.pos[i, 1]) / self.cfg.world_h,
                (self.pos[i, 1] + hy) / self.cfg.world_h,
            ], dtype=np.float32)

            self_state = np.array([
                self.pos[i, 0] / hx,
                self.pos[i, 1] / hy,
                self.vel[i, 0] / max(1e-6, self.cfg.max_speed),
                self.vel[i, 1] / max(1e-6, self.cfg.max_speed),
                math.sin(float(self.heading[i])),
                math.cos(float(self.heading[i])),
                *wall_dists.tolist(),
                self.step_count / max(1, self.max_steps),
                float(np.mean(coverage)),
                float(np.min(coverage)),
                float(np.max(coverage)),
                float(np.mean(never_crop)),
            ], dtype=np.float32)

            local_obs.append({'maps': maps, 'self_state': self_state})
        return local_obs

    def get_centralized_state(self) -> np.ndarray:
        coverage_small = self._downsample(self.coverage_value, 4, 4)
        never_small = self._downsample(1.0 - self.ever_visited.astype(np.float32), 4, 4)
        drones = np.zeros((self.cfg.grid_h, self.cfg.grid_w), dtype=np.float32)
        ix, iy = self._world_to_grid(self.pos)
        drones[iy, ix] = 1.0
        drones_small = self._downsample(drones, 4, 4)
        active_small = self._downsample(self._active_area_mask().astype(np.float32), 4, 4)

        hx = self.cfg.world_w / 2.0
        hy = self.cfg.world_h / 2.0
        joint = []
        for i in range(self.n_agents):
            boundary_min = min(hx - self.pos[i,0], self.pos[i,0] + hx, hy - self.pos[i,1], self.pos[i,1] + hy)
            joint.extend([
                self.pos[i,0] / hx,
                self.pos[i,1] / hy,
                self.vel[i,0] / max(1e-6, self.cfg.max_speed),
                self.vel[i,1] / max(1e-6, self.cfg.max_speed),
                math.sin(float(self.heading[i])),
                math.cos(float(self.heading[i])),
                boundary_min / max(1e-6, min(hx, hy)),
            ])

        extras = [
            self.step_count / max(1, self.max_steps),
            float(np.mean(self.coverage_value)),
            float(np.mean(self.ever_visited)),
            float(np.mean(self.coverage_value > 0.5)),
            float(self.time_to_50 if self.time_to_50 >= 0 else -1.0),
            float(self.time_to_80 if self.time_to_80 >= 0 else -1.0),
            float(self.time_to_95 if self.time_to_95 >= 0 else -1.0),
        ]
        return np.concatenate([
            coverage_small.flatten(),
            never_small.flatten(),
            drones_small.flatten(),
            active_small.flatten(),
            np.asarray(joint, dtype=np.float32),
            np.asarray(extras, dtype=np.float32),
        ]).astype(np.float32)

    def _downsample(self, arr: np.ndarray, out_h: int, out_w: int) -> np.ndarray:
        ys = np.linspace(0, arr.shape[0], out_h + 1, dtype=int)
        xs = np.linspace(0, arr.shape[1], out_w + 1, dtype=int)
        out = np.zeros((out_h, out_w), dtype=np.float32)
        for i in range(out_h):
            for j in range(out_w):
                patch = arr[ys[i]:ys[i+1], xs[j]:xs[j+1]]
                out[i,j] = float(np.mean(patch)) if patch.size else 0.0
        return out

    def compute_scan_efficiency_score(self) -> float:
        overlap = float(np.mean(self._overlap_ratio()))
        coll = float(np.mean(self._collision_stats()[0]))
        return float(
            0.60 * float(np.mean(self.ever_visited))
            + 0.20 * float(np.mean(self.coverage_value))
            + 0.15 * max(0.0, 1.0 - overlap)
            - 0.05 * coll
        )

    def _build_info(self, rewards: np.ndarray) -> Dict[str, float]:
        return {
            'reward_mean': float(np.mean(rewards)),
            'coverage_mean': float(np.mean(self.coverage_value)),
            'ever_seen_fraction': float(np.mean(self.ever_visited)),
            'maintained_fraction': float(np.mean(self.coverage_value > 0.5)),
            'new_area_delta': float(self.last_ever_seen_fraction),
            'step': float(self.step_count),
            'time_to_50_coverage': float(self.time_to_50),
            'time_to_80_coverage': float(self.time_to_80),
            'time_to_95_coverage': float(self.time_to_95),
            'scan_efficiency_score': self.compute_scan_efficiency_score(),
        }

    def render_state(self) -> Dict[str, np.ndarray]:
        return {
            'coverage_value': self.coverage_value.copy(),
            'ever_visited': self.ever_visited.copy(),
            'positions': self.pos.copy(),
            'velocities': self.vel.copy(),
            'headings': self.heading.copy(),
            'visited_by_mask': self.visited_by_mask.copy(),
            'active_area': self._active_area_mask().astype(np.uint8),
        }
