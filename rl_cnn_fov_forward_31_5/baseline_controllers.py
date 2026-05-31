from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class _ControllerState:
    x_dir: np.ndarray | None = None
    y_targets: np.ndarray | None = None


class FrontierHeuristicController:
    """Greedy frontier-seeking controller.

    Each active drone gets a sector of the polygon around the centroid and
    moves toward the nearest uncovered cell in its sector. If its sector is
    already covered, it falls back to the nearest uncovered cell globally.
    Repulsion from nearby drones is added to reduce collisions.
    """

    def __init__(self, max_speed: float, world_w: float, world_h: float):
        self.max_speed = max_speed
        self.world_scale = max(world_w, world_h)

    def act(self, env) -> np.ndarray:
        actions = np.zeros((env.max_agents, 3), dtype=np.float32)
        active_idxs = np.where(env.active_mask > 0.5)[0]
        if len(active_idxs) == 0:
            return actions

        uncovered = env.active_area_mask & (env.ever_visited == 0)
        if np.any(uncovered):
            target_pts = env.grid_points[uncovered]
        else:
            # Once everything has been covered at least once, maintain stale / low-coverage regions.
            low_cov = env.active_area_mask & (env.coverage_value < env.cfg.maintained_threshold)
            target_pts = env.grid_points[low_cov] if np.any(low_cov) else env.grid_points[env.active_area_mask]

        centroid = np.mean(env.polygon_vertices, axis=0)
        rel = target_pts - centroid[None, :]
        target_angles = np.arctan2(rel[:, 1], rel[:, 0])

        drone_angles = np.arctan2(env.pos[active_idxs, 1] - centroid[1], env.pos[active_idxs, 0] - centroid[0])
        order = np.argsort(drone_angles)
        ordered_active = active_idxs[order]

        # Divide by angular sector.
        sector_edges = np.linspace(-math.pi, math.pi, len(ordered_active) + 1)

        for k, i in enumerate(ordered_active):
            lo, hi = sector_edges[k], sector_edges[k + 1]
            in_sector = (target_angles >= lo) & (target_angles <= hi)
            sector_targets = target_pts[in_sector]
            if len(sector_targets) == 0:
                sector_targets = target_pts
            diffs = sector_targets - env.pos[i][None, :]
            dists = np.linalg.norm(diffs, axis=1)
            tgt = sector_targets[np.argmin(dists)]
            actions[i] = _move_toward_with_repulsion(env, i, tgt, self.max_speed)
        return actions


class LawnmowerHeuristicController:
    """Simple boustrophedon/strip sweep baseline.

    Assign each drone one or more horizontal stripes inside the polygon.
    The drone moves left-right within its current stripe, then shifts to the
    next stripe. This creates a hand-crafted full-area coverage baseline.
    """

    def __init__(self, max_speed: float, world_w: float, world_h: float, stripe_spacing: float = 4.0):
        self.max_speed = max_speed
        self.world_scale = max(world_w, world_h)
        self.stripe_spacing = stripe_spacing
        self.state = _ControllerState()

    def reset(self, env) -> None:
        active_idxs = np.where(env.active_mask > 0.5)[0]
        if len(active_idxs) == 0:
            self.state.x_dir = None
            self.state.y_targets = None
            return
        min_x, max_x = np.min(env.polygon_vertices[:, 0]), np.max(env.polygon_vertices[:, 0])
        min_y, max_y = np.min(env.polygon_vertices[:, 1]), np.max(env.polygon_vertices[:, 1])
        stripe_ys = np.arange(min_y + 1.0, max_y - 1.0 + 1e-6, self.stripe_spacing)
        if len(stripe_ys) == 0:
            stripe_ys = np.array([0.5 * (min_y + max_y)])
        per_drone = [[] for _ in active_idxs]
        for idx, y in enumerate(stripe_ys):
            per_drone[idx % len(active_idxs)].append(y)
        y_targets = np.zeros((env.max_agents,), dtype=np.float32)
        x_dir = np.ones((env.max_agents,), dtype=np.float32)
        for slot, i in enumerate(active_idxs):
            if per_drone[slot]:
                y_targets[i] = per_drone[slot][0]
            else:
                y_targets[i] = 0.5 * (min_y + max_y)
            x_dir[i] = 1.0 if (slot % 2 == 0) else -1.0
        self.state.x_dir = x_dir
        self.state.y_targets = y_targets

    def act(self, env) -> np.ndarray:
        if self.state.x_dir is None or self.state.y_targets is None:
            self.reset(env)
        actions = np.zeros((env.max_agents, 3), dtype=np.float32)
        active_idxs = np.where(env.active_mask > 0.5)[0]
        if len(active_idxs) == 0:
            return actions
        min_x, max_x = np.min(env.polygon_vertices[:, 0]), np.max(env.polygon_vertices[:, 0])
        min_y, max_y = np.min(env.polygon_vertices[:, 1]), np.max(env.polygon_vertices[:, 1])
        for i in active_idxs:
            x_dir = self.state.x_dir[i]
            y_target = self.state.y_targets[i]
            # When near left/right edge, reverse x and move stripe upward.
            near_left = env.pos[i, 0] < min_x + env.cfg.wall_safety_margin + 1.0
            near_right = env.pos[i, 0] > max_x - env.cfg.wall_safety_margin - 1.0
            if (x_dir < 0 and near_left) or (x_dir > 0 and near_right):
                self.state.x_dir[i] *= -1.0
                y_target = min(max_y - env.cfg.wall_safety_margin, y_target + self.stripe_spacing)
                self.state.y_targets[i] = y_target if y_target <= max_y else min_y + 1.0
                x_dir = self.state.x_dir[i]
            tgt = np.array([max_x - 1.0 if x_dir > 0 else min_x + 1.0, self.state.y_targets[i]], dtype=np.float32)
            actions[i] = _move_toward_with_repulsion(env, i, tgt, self.max_speed)
        return actions


def _move_toward_with_repulsion(env, i: int, target: np.ndarray, max_speed: float) -> np.ndarray:
    diff = target - env.pos[i]
    dist = float(np.linalg.norm(diff))
    heading = float(env.heading[i])
    desired = diff / max(1e-6, dist)
    # Add repulsion from nearby active drones.
    repulse = np.zeros((2,), dtype=np.float32)
    for j in np.where(env.active_mask > 0.5)[0]:
        if i == j:
            continue
        rel = env.pos[i] - env.pos[j]
        d = float(np.linalg.norm(rel))
        if d < 2.5 * env.cfg.drone_safety_radius:
            repulse += rel / max(1e-6, d) * max(0.0, (2.5 * env.cfg.drone_safety_radius - d) / (2.5 * env.cfg.drone_safety_radius))
    desired = desired + 0.8 * repulse
    if np.linalg.norm(desired) < 1e-6:
        desired = np.array([math.cos(heading), math.sin(heading)], dtype=np.float32)
    desired = desired / max(1e-6, np.linalg.norm(desired))
    desired_heading = math.atan2(float(desired[1]), float(desired[0]))
    dtheta = (desired_heading - heading + math.pi) % (2.0 * math.pi) - math.pi
    turn_cmd = np.clip(dtheta / math.radians(90.0), -1.0, 1.0)
    forward = np.array([math.cos(heading), math.sin(heading)], dtype=np.float32)
    lateral = np.array([-math.sin(heading), math.cos(heading)], dtype=np.float32)
    speed_cmd = np.clip(np.dot(desired, forward), 0.1, 1.0)
    strafe_cmd = np.clip(np.dot(desired, lateral), -0.6, 0.6)
    # Map speed command [0,1] back to action first component [-1,1]
    a0 = np.clip(2.0 * speed_cmd - 1.0, -1.0, 1.0)
    return np.array([a0, turn_cmd, strafe_cmd], dtype=np.float32)
