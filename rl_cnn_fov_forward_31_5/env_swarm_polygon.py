from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from polygon_utils import (
    distance_to_polygon_edge,
    distance_to_polygon_edge_batch,
    generate_concave_polygon,
    points_in_polygon,
    polygon_area,
    polygon_bbox,
    sample_separated_points_in_polygon,
)

@dataclass
class EnvConfig:
    max_agents: int = 5
    min_active_agents: int = 5
    randomize_active_agents: bool = False
    allow_mid_episode_failures: bool = False
    failure_probability_per_second: float = 0.0

    decision_hz: float = 10.0
    episode_seconds: float = 240.0     # increased from 180s — slower speed (2.5m/s) needs more time for thorough coverage
    target_first_cover_fraction: float = 0.98

    world_w: float = 40.0
    world_h: float = 40.0
    polygon_num_vertices: int = 12
    polygon_target_area: float = 900.0
    polygon_margin: float = 1.5

    grid_resolution_m: float = 0.5     # pixel size for ALL map channels (Ch0-Ch5) and self-state[8-12]. Local map = 32×32 × 0.5m = 16m×16m window
    local_map_size: int = 32           # local map crop size in pixels → feeds CNN (32×32 × 6 channels)
    critic_downsample: int = 8         # critic global map downsampling factor

    max_speed: float = 2.5             # reduced from 3.5 — prevents gaps between scan sweeps. At 2.5m/s + 10Hz = 0.25m/step, 90° FOV covers ~3.5m width at 2.5m range → no gaps
    max_turn_rate_deg: float = 360.0   # physics: max heading change per step
    max_strafe_ratio: float = 0.30     # physics: sideways speed limit relative to forward

    drone_safety_radius: float = 1.5   # RUNTIME: center-to-center min distance (hard enforced + penalty). SAFETY CRITICAL — increased from 1.0
    wall_safety_margin: float = 1.0    # RUNTIME: wall penalty starts at this distance from polygon edge. SAFETY CRITICAL
    coverage_wall_margin: float = 1.5  # INIT: cells within 1.5m of wall are DEAD ZONE — not coverable, not counted as targets
    spawn_pair_distance: float = 3.0   # INIT (reset only): drones spawn at least this far apart
    spawn_wall_margin: float = 3.0     # INIT (reset only): minimum distance from polygon edge when spawning

    # ── Sensor parameters ──
    sensor_forward_range_m: float = 2.5  # Ch4 only: range of the forward-sensor FOV cone visualization
    scan_range_m: float = 2.5           # Ch0,Ch1 + reward: max distance at which drone marks cells as covered. With min_edge_keep=1.2, can reach cells 0.3m from wall
    scan_fov_deg: float = 90.0          # Ch0,Ch1 + reward: WIDENED from 60° to 90° to reduce gaps between scan sweeps
    forward_sensor_fov_deg: float = 90.0 # Ch4 only: match scan_fov_deg
    corner_scan_distance: float = 3.5    # self-state[16] only: corner_proximity = 1 - (dist_to_nearest_vertex / this). Reduced from 5.0 for tighter corner targeting
    trajectory_heatmap_decay: float = 0.97 # Ch5 + self-state[17]: decay rate for trajectory heatmap per step

    decay_reset_seconds: float = 60.0   # faster decay — forces revisits (was 120, too slow)
    maintained_threshold: float = 0.30  # easier threshold — more cells count as "maintained"

    # ── Maintenance NEED parameters ──
    maint_need_decay_tau: float = 40.0     # time constant (seconds) for exponential need growth after last scan
    maint_geometry_corner_boost: float = 2.0  # multiplicative boost for cells near polygon corners
    maint_geometry_edge_boost: float = 1.5    # multiplicative boost for cells near polygon edges
    maint_geometry_radius_m: float = 3.0      # radius for corner/edge proximity influence
    maint_diminishing_half_life: float = 5.0  # seconds: repeated scans halve the marginal gain
    w_maint_need_reduction: float = 150.0     # reward weight for need-reduction-based maintenance gain

    # ── POSITIVE rewards (should dominate the gradient) ──
    w_first_visit: float = 600.0       # team: cells seen for the first time (biggest driver)
    w_ever_seen_gain: float = 800.0    # team: ever-seen fraction increase
    w_refresh_gain: float = 80.0       # team: re-covering stale cells (was 10 — way too low for maintenance)
    w_maintained_gain: float = 40.0    # team: maintaining coverage (was 5 — too low)
    w_frontier_progress: float = 0.0    # REMOVED: was directing actor explicitly → now learned via critic
    w_new_cells: float = 500.0         # per-drone: fraction of scan cone that was new (CRITICAL for spreading)
    w_frontier_align: float = 0.0      # REMOVED: was directing actor explicitly → now learned via critic
    w_spread: float = 250.0            # per-drone: spatial separation from other drones (increased to force spreading)
    w_displacement: float = 15.0       # per-drone: net displacement (low to avoid rewarding circles)
    w_heading_spread: float = 60.0     # per-drone: heading diversity — more important with 60° scan
    w_alive: float = 0.5               # per-drone: small baseline for being active
    w_corner_bonus: float = 400.0      # per-drone: reward for scanning near polygon vertices (corners) — doubled to fix corner gaps
    w_team_uncovered_penalty: float = 30.0  # stronger per-step pressure to cover remaining cells (doubled)
    w_team_stale_penalty: float = 15.0      # stronger pressure to refresh stale cells
    # ── NEGATIVE penalties (soft guards, NOT the main learning signal) ──
    w_collision: float = 500.0         # physical collision — MUST NOT HAPPEN. SAFETY ABOVE ALL! Dominates any positive reward
    w_wall: float = 500.0              # wall approach — MUST NOT HAPPEN. SAFETY ABOVE ALL! Dominates any positive reward
    w_revisit: float = 15.0            # penalize scanning already-covered cells (strong in Phase 1)
    w_circling: float = 80.0           # circling detection — MUST be strong enough to break loops
    w_sustained_turn: float = 40.0     # penalty for turning in the same direction too many consecutive steps
    w_stagnation: float = 30.0         # not moving / barely moving
    w_overlap: float = 50.0            # scan overlap with other drones (strongly penalize redundant coverage)
    w_local_staleness: float = 80.0    # lingering in well-covered area — MUST LEAVE during Phase 1
    w_scan_gap: float = 40.0           # moving fast while leaving unseen cells nearby (prevents holes)
    w_frontier_dist: float = 20.0      # penalty for being far from nearest uncovered cell (scales up in late-game)
    w_fairness: float = 100.0          # penalty for drones contributing far below average — MUST contribute equally
    w_late_hunt: float = 500.0         # massive bonus in late game (>93%) for finding remaining cells
    w_control: float = 0.02            # action smoothness (tiny)
    w_local_responsibility: float = 200.0  # per-drone: reward covering cells you're the closest drone to
    w_hole_left_behind: float = 300.0      # per-drone: penalty for moving AWAY from nearby uncovered cells
    # Milestone bonuses (one-time, shared by all drones)
    w_milestone_50: float = 50.0
    w_milestone_80: float = 150.0
    w_milestone_90: float = 300.0
    w_milestone_95: float = 500.0
    w_milestone_98: float = 1000.0
    w_milestone_100: float = 2000.0


class SwarmSearchPolygonEnv:
    def __init__(self, cfg: EnvConfig | None = None):
        self.cfg = cfg or EnvConfig()
        self.max_agents = self.cfg.max_agents
        self.dt = 1.0 / self.cfg.decision_hz
        self.max_steps = int(round(self.cfg.episode_seconds * self.cfg.decision_hz))
        self.grid_w = int(round(self.cfg.world_w / self.cfg.grid_resolution_m))
        self.grid_h = int(round(self.cfg.world_h / self.cfg.grid_resolution_m))
        self.local_map_size = self.cfg.local_map_size
        self.local_radius = self.local_map_size // 2

        self.pos = np.zeros((self.max_agents, 2), dtype=np.float32)
        self.prev_pos = np.zeros((self.max_agents, 2), dtype=np.float32)
        self.vel = np.zeros((self.max_agents, 2), dtype=np.float32)
        self.heading = np.zeros((self.max_agents,), dtype=np.float32)
        self.prev_actions = np.zeros((self.max_agents, 3), dtype=np.float32)
        self.active_mask = np.zeros((self.max_agents,), dtype=np.float32)

        self.coverage_age = np.full((self.grid_h, self.grid_w), self.cfg.decay_reset_seconds, dtype=np.float32)
        self.coverage_value = np.zeros((self.grid_h, self.grid_w), dtype=np.float32)
        self.ever_visited = np.zeros((self.grid_h, self.grid_w), dtype=np.uint8)
        self._ever_visited_before = np.zeros((self.grid_h, self.grid_w), dtype=np.uint8)
        self.last_owner = -np.ones((self.grid_h, self.grid_w), dtype=np.int16)

        self.polygon_vertices = np.zeros((self.cfg.polygon_num_vertices, 2), dtype=np.float32)
        self.active_area_mask = np.zeros((self.grid_h, self.grid_w), dtype=bool)
        self.step_count = 0
        self.total_dropped_agents = 0
        self._cached_scan_masks = None
        self._cached_scan_step = -1
        self._cached_frontier_map = None
        self._cached_frontier_map_step = -1
        self.time_to_50 = -1.0
        self.time_to_80 = -1.0
        self.time_to_95 = -1.0
        self.last_ever_seen_fraction = 0.0
        self.last_maintained_fraction = 0.0
        self._milestone_hit = {50: False, 80: False, 90: False, 95: False, 98: False, 100: False}

        # ── displacement tracking (rolling window) ──
        self._displacement_window = 80  # ~8 s at 10 Hz — catches wide circling patterns
        self._pos_history = np.zeros((self.max_agents, self._displacement_window, 2), dtype=np.float32)
        self._pos_hist_ptr = 0
        self._drone_discoveries = np.zeros((self.max_agents,), dtype=np.int32)
        self._step_discoveries = np.zeros((self.max_agents,), dtype=np.int32)

        # ── sustained turn tracking (anti-circling) ──
        self._turn_sign_streak = np.zeros((self.max_agents,), dtype=np.int32)  # consecutive steps with same turn sign

        # ── trajectory heatmap (communicated between drones) ──
        self._trajectory_heatmap = np.zeros((self.grid_h, self.grid_w), dtype=np.float32)
        # per-drone trajectory heatmaps for individual path tracking
        self._drone_trajectory_maps = np.zeros((self.max_agents, self.grid_h, self.grid_w), dtype=np.float32)

        # ── SAFETY tracking (hard gate) ──
        self._episode_collision_count = 0    # drone-drone safety radius violations
        self._episode_wall_collision_count = 0  # hard wall breaches (outside polygon or edge < min_edge_keep)
        self._safety_violated = False        # once True, episode score = 0

        # ── Recent contribution sliding window for fairness ──
        self._fairness_window = 200  # ~20s at 10Hz
        self._recent_discoveries = np.zeros((self.max_agents, self._fairness_window), dtype=np.int32)
        self._recent_disc_ptr = 0

        # ── Maintenance NEED tracking ──
        # Geometry weight map: precomputed per polygon, higher near corners/edges
        self._maint_geometry_weight = np.ones((self.grid_h, self.grid_w), dtype=np.float32)
        # Per-cell recent scan accumulator: tracks diminishing returns (exponential decay)
        # Higher value = cell was recently scanned many times → less marginal gain
        self._maint_recent_scans = np.zeros((self.grid_h, self.grid_w), dtype=np.float32)
        # Per-cell maintenance need (computed each step): risk score [0, ∞)
        self._maint_need_map = np.zeros((self.grid_h, self.grid_w), dtype=np.float32)
        # Snapshot of need map BEFORE coverage update (for gain computation)
        self._maint_need_before = np.zeros((self.grid_h, self.grid_w), dtype=np.float32)
        # Per-drone maintenance gain this step
        self._step_maint_gain = np.zeros((self.max_agents,), dtype=np.float32)
        # Per-drone cumulative maintenance gain (for evaluation)
        self._drone_maint_gains = np.zeros((self.max_agents,), dtype=np.float32)

        self._rng = np.random.default_rng(0)
        xs = (np.arange(self.grid_w, dtype=np.float32) + 0.5) * self.cfg.grid_resolution_m - self.cfg.world_w / 2.0
        ys = (np.arange(self.grid_h, dtype=np.float32) + 0.5) * self.cfg.grid_resolution_m - self.cfg.world_h / 2.0
        self.grid_x, self.grid_y = np.meshgrid(xs, ys)
        self.grid_points = np.stack([self.grid_x, self.grid_y], axis=-1)

        # Precompute forward-sensor pixel grids (reused every step)
        s = self.local_map_size
        c = s // 2
        _yy, _xx = np.mgrid[0:s, 0:s]
        self._fwd_dx = (_xx - c).astype(np.float32) * self.cfg.grid_resolution_m
        self._fwd_dy = (_yy - c).astype(np.float32) * self.cfg.grid_resolution_m
        self._fwd_dist = np.sqrt(self._fwd_dx ** 2 + self._fwd_dy ** 2)
        self._fwd_ang = np.arctan2(self._fwd_dy, self._fwd_dx)
        self._fwd_range_mask = (self._fwd_dist > 1e-9) & (self._fwd_dist <= self.cfg.sensor_forward_range_m)

    @property
    def map_channels(self) -> int:
        return 1  # TRUE 1-Channel Master Map (Terrain + Coverage)

    @property
    def self_state_dim(self) -> int:
        return 21  # +nearest_uncov_dist, +local_uncov_frac, +recent_contrib_rate

    @property
    def neighbor_state_dim(self) -> int:
        return 10  # +heading_sin, +heading_cos, +traj_dir_x, +traj_dir_y

    @property
    def max_neighbors(self) -> int:
        return self.max_agents - 1

    @property
    def actor_obs_spec(self) -> dict[str, int]:
        return {
            'map_channels': self.map_channels,
            'map_size': self.local_map_size,
            'self_state_dim': self.self_state_dim,
            'neighbor_state_dim': self.neighbor_state_dim,
            'max_neighbors': self.max_neighbors,
        }

    @property
    def critic_obs_spec(self) -> dict[str, int]:
        ds = self.cfg.critic_downsample
        # Add per-corner coverage features so the critic can explicitly reason
        # about polygon corners and encourage full corner coverage. The number
        # of corners is fixed for a given environment instance (polygon_vertices).
        num_corners = len(self.polygon_vertices)
        # We include TWO features per corner:
        #  - local mean coverage_value around the corner (0..1)
        #  - fraction of cells in the neighborhood that were ever visited (0..1)
        corner_feat_per = 2
        return {
            'map_channels': 5,
            'map_size': ds,
            'global_feature_dim': self.max_agents * 7 + 13 + num_corners * corner_feat_per + self.max_agents + 2,
            'num_corners': num_corners,
            'corner_feat_per': corner_feat_per,
        }

    @property
    def critic_state_dim(self) -> int:
        spec = self.critic_obs_spec
        return spec['map_channels'] * spec['map_size'] * spec['map_size'] + spec['global_feature_dim']

    def reset(self, seed: int | None = None):
        if seed is not None:
            self._rng = np.random.default_rng(seed)
        self.step_count = 0
        self.total_dropped_agents = 0
        self.coverage_age.fill(self.cfg.decay_reset_seconds)
        self.coverage_value.fill(0.0)
        self.ever_visited.fill(0)
        self.last_owner.fill(-1)
        self.vel.fill(0.0)
        self.prev_actions.fill(0.0)
        self.last_ever_seen_fraction = 0.0
        self.last_maintained_fraction = 0.0
        self.time_to_50 = -1.0
        self.time_to_80 = -1.0
        self.time_to_95 = -1.0
        self._milestone_hit = {50: False, 80: False, 90: False, 95: False, 98: False, 100: False}
        self._drone_discoveries = np.zeros((self.max_agents,), dtype=np.int32)  # per-drone new cell count
        self._pos_history.fill(0.0)
        self._pos_hist_ptr = 0
        self._turn_sign_streak.fill(0)
        self._trajectory_heatmap.fill(0.0)
        self._drone_trajectory_maps.fill(0.0)
        self._episode_collision_count = 0
        self._episode_wall_collision_count = 0
        self._safety_violated = False
        self._recent_discoveries.fill(0)
        self._recent_disc_ptr = 0
        self._maint_recent_scans.fill(0.0)
        self._maint_need_map.fill(0.0)
        self._maint_need_before.fill(0.0)
        self._step_maint_gain.fill(0.0)
        self._drone_maint_gains.fill(0.0)
        self._generate_new_polygon()
        self._spawn_agents()
        self.prev_pos = self.pos.copy()
        self._update_coverage()
        obs = self.get_actor_observation()
        info = self._build_info(np.zeros((self.max_agents,), dtype=np.float32))
        return obs, info

    def _generate_new_polygon(self):
        for _ in range(200):
            vertices = generate_concave_polygon(
                num_vertices=self.cfg.polygon_num_vertices,
                target_area=self.cfg.polygon_target_area,
                rng=self._rng,
            ).astype(np.float32)
            min_x, max_x, min_y, max_y = polygon_bbox(vertices)
            if (
                min_x >= -self.cfg.world_w / 2.0 + self.cfg.polygon_margin and
                max_x <= self.cfg.world_w / 2.0 - self.cfg.polygon_margin and
                min_y >= -self.cfg.world_h / 2.0 + self.cfg.polygon_margin and
                max_y <= self.cfg.world_h / 2.0 - self.cfg.polygon_margin
            ):
                self.polygon_vertices = vertices
                break
        else:
            side = math.sqrt(self.cfg.polygon_target_area)
            half = min(side / 2.0, self.cfg.world_w / 2.0 - 2.0, self.cfg.world_h / 2.0 - 2.0)
            self.polygon_vertices = np.array([[-half, -half], [half, -half], [half, half], [-half, half]], dtype=np.float32)
        inside_mask = points_in_polygon(self.grid_points, self.polygon_vertices)
        # Exclude cells within coverage_wall_margin of the polygon edge (border insurance)
        flat_pts = self.grid_points.reshape(-1, 2)
        edge_dists = distance_to_polygon_edge_batch(flat_pts, self.polygon_vertices).reshape(self.grid_h, self.grid_w)
        self.active_area_mask = inside_mask & (edge_dists >= self.cfg.coverage_wall_margin)
        self.coverage_value[~self.active_area_mask] = 0.0
        self.coverage_age[~self.active_area_mask] = self.cfg.decay_reset_seconds
        self.ever_visited[~self.active_area_mask] = 0
        self.last_owner[~self.active_area_mask] = -1

        # ── Precompute geometry-aware maintenance weight map ──
        # Cells near polygon corners and edges have higher maintenance need
        # because they are harder to reach and more prone to being neglected.
        geo_w = np.ones((self.grid_h, self.grid_w), dtype=np.float32)
        radius_m = self.cfg.maint_geometry_radius_m
        # Corner boost: cells near polygon vertices
        for v in self.polygon_vertices:
            dist_to_v = np.linalg.norm(self.grid_points - v[None, None, :], axis=-1)
            proximity = np.clip(1.0 - dist_to_v / radius_m, 0.0, 1.0)
            geo_w += proximity * (self.cfg.maint_geometry_corner_boost - 1.0)
        # Edge boost: cells near polygon boundary (but not dead zone)
        edge_proximity = np.clip(1.0 - edge_dists / radius_m, 0.0, 1.0)
        geo_w += edge_proximity * (self.cfg.maint_geometry_edge_boost - 1.0)
        # Only active cells have geometry weight
        geo_w[~self.active_area_mask] = 0.0
        self._maint_geometry_weight = geo_w

    def _spawn_agents(self):
        n_active = int(self._rng.integers(self.cfg.min_active_agents, self.cfg.max_agents + 1)) if self.cfg.randomize_active_agents else self.cfg.max_agents
        self.active_mask.fill(0.0)
        self.active_mask[:n_active] = 1.0
        pts = sample_separated_points_in_polygon(
            self.polygon_vertices,
            count=n_active,
            min_edge_distance=self.cfg.spawn_wall_margin,
            min_pair_distance=max(self.cfg.spawn_pair_distance, self.cfg.drone_safety_radius + 0.5),
            rng=self._rng,
        )
        self.pos.fill(0.0)
        self.pos[:n_active] = pts
        self.vel.fill(0.0)
        self.heading.fill(0.0)
        self.heading[:n_active] = self._rng.uniform(-math.pi, math.pi, size=(n_active,)).astype(np.float32)
        perm = self._rng.permutation(self.max_agents)
        self.pos = self.pos[perm]
        self.prev_pos = self.prev_pos[perm]
        self.vel = self.vel[perm]
        self.heading = self.heading[perm]
        self.prev_actions = self.prev_actions[perm]
        self.active_mask = self.active_mask[perm]

    def step(self, actions: np.ndarray):
        actions = np.asarray(actions, dtype=np.float32)
        if actions.shape != (self.max_agents, 3):
            raise ValueError(f'Expected {(self.max_agents, 3)}, got {actions.shape}')
        active_before = self.active_mask.copy()
        self.prev_pos = self.pos.copy()
        clipped = np.clip(actions, -1.0, 1.0)
        speed = ((clipped[:, 0] + 1.0) * 0.5) * self.cfg.max_speed
        turn = clipped[:, 1] * math.radians(self.cfg.max_turn_rate_deg)
        strafe = clipped[:, 2] * self.cfg.max_strafe_ratio * self.cfg.max_speed

        self.heading += turn * self.dt * active_before
        self.heading = ((self.heading + math.pi) % (2.0 * math.pi)) - math.pi

        # ── Track sustained turning (same-sign turn commands) ──
        for i in np.where(active_before > 0.5)[0]:
            if abs(clipped[i, 1]) < 0.05:  # near-zero turn = reset streak
                self._turn_sign_streak[i] = 0
            elif (clipped[i, 1] > 0) == (self._turn_sign_streak[i] >= 0):
                self._turn_sign_streak[i] += (1 if clipped[i, 1] > 0 else -1)
            else:
                self._turn_sign_streak[i] = (1 if clipped[i, 1] > 0 else -1)

        forward = np.stack([np.cos(self.heading), np.sin(self.heading)], axis=1)
        lateral = np.stack([-np.sin(self.heading), np.cos(self.heading)], axis=1)
        self.vel = (forward * speed[:, None] + lateral * strafe[:, None]) * active_before[:, None]
        proposed = self.pos + self.vel * self.dt

        hard_wall = np.zeros((self.max_agents,), dtype=np.float32)
        min_edge_keep = 0.8  # SAFETY CRITICAL: minimum distance from polygon edge. Drones MUST NOT cross this.
        centroid = np.mean(self.polygon_vertices, axis=0)

        # ── LOCAL FINISH CONSTRAINT ──
        # If coverage >= 95% and there are uncovered cells within 2× scan_range,
        # clamp speed to max_speed * 0.5 and bias heading toward nearest uncovered cell.
        # This is a HARD behavioral rule, not a soft reward.
        es = self.last_ever_seen_fraction
        if es >= 0.95:
            for i in np.where(active_before > 0.5)[0]:
                uncov_count, nearest_uncov_dir, nearest_uncov_dist = self._uncovered_near_agent(i)
                if uncov_count > 0 and nearest_uncov_dist < self.cfg.scan_range_m * 2.5:
                    # Slow down — don't run past uncovered cells
                    speed[i] = min(speed[i], self.cfg.max_speed * 0.4)
                    # If uncovered cell is behind the drone (dot product with heading < -0.3),
                    # force a turn toward it
                    fwd = np.array([math.cos(float(self.heading[i])), math.sin(float(self.heading[i]))], dtype=np.float32)
                    alignment = float(np.dot(fwd, nearest_uncov_dir))
                    if alignment < 0.0:
                        # Force turn toward uncovered cell: override turn command
                        cross = fwd[0] * nearest_uncov_dir[1] - fwd[1] * nearest_uncov_dir[0]
                        forced_turn = math.copysign(1.0, cross) * math.radians(self.cfg.max_turn_rate_deg) * 0.8
                        turn[i] = forced_turn

        for i in range(self.max_agents):
            if active_before[i] <= 0.0:
                continue
            inside = bool(points_in_polygon(proposed[i][None, None, :], self.polygon_vertices)[0, 0])
            edge_dist = distance_to_polygon_edge(proposed[i], self.polygon_vertices)
            if (not inside) or (edge_dist < min_edge_keep):
                # Slide along wall: push drone inward instead of freezing it
                inward = centroid - self.pos[i]
                inward_len = np.linalg.norm(inward)
                if inward_len > 1e-6:
                    inward_dir = inward / inward_len
                else:
                    inward_dir = np.array([0.0, 0.0], dtype=np.float32)
                # Project velocity onto tangent (perpendicular to inward)
                tangent = np.array([-inward_dir[1], inward_dir[0]], dtype=np.float32)
                slide_vel = np.dot(self.vel[i], tangent) * tangent
                # Push slightly inward + slide along wall
                push = inward_dir * self.cfg.max_speed * 0.5 * self.dt
                new_pos = self.pos[i] + slide_vel * self.dt + push
                new_inside = bool(points_in_polygon(new_pos[None, None, :], self.polygon_vertices)[0, 0])
                new_edge = distance_to_polygon_edge(new_pos, self.polygon_vertices)
                if new_inside and new_edge >= min_edge_keep:
                    proposed[i] = new_pos
                    self.vel[i] = slide_vel
                else:
                    # Fallback: push inward more aggressively
                    proposed[i] = self.pos[i] + push * 2.0
                    fallback_inside = bool(points_in_polygon(proposed[i][None, None, :], self.polygon_vertices)[0, 0])
                    if not fallback_inside:
                        proposed[i] = self.pos[i]
                    self.vel[i] = 0.0
                hard_wall[i] = 1.0

            # ── HARD CLAMP: absolutely guarantee drone stays inside polygon ──
            final_inside = bool(points_in_polygon(proposed[i][None, None, :], self.polygon_vertices)[0, 0])
            if not final_inside:
                # Emergency: snap back to previous position (which was inside)
                proposed[i] = self.pos[i] if bool(points_in_polygon(self.pos[i][None, None, :], self.polygon_vertices)[0, 0]) else centroid.copy()
                self.vel[i] = 0.0
                hard_wall[i] = 1.0

            self.pos[i] = proposed[i]

        # ── Hard collision avoidance: push drones apart if too close ──
        # SAFETY CRITICAL: drones MUST maintain drone_safety_radius separation
        active_idxs = np.where(active_before > 0.5)[0]
        min_sep = self.cfg.drone_safety_radius  # minimum allowed center-to-center (1.5m)
        for _ in range(5):  # more iterations to resolve chain collisions
            for i in range(len(active_idxs)):
                for j in range(i + 1, len(active_idxs)):
                    ai, aj = active_idxs[i], active_idxs[j]
                    diff = self.pos[ai] - self.pos[aj]
                    dist = np.linalg.norm(diff)
                    if dist < min_sep:
                        if dist < 1e-6:
                            diff = np.array([1.0, 0.0], dtype=np.float32)
                            dist = 1e-6
                        # Push each drone half the overlap distance apart + extra margin
                        overlap = min_sep - dist
                        push_dir = diff / dist
                        push_amount = overlap / 2 + 0.1  # extra 10cm safety margin
                        self.pos[ai] += push_dir * push_amount
                        self.pos[aj] -= push_dir * push_amount

        # ── Re-validate wall safety after collision push-apart ──
        # Collision resolution may have pushed drones outside polygon
        for i in active_idxs:
            inside = bool(points_in_polygon(self.pos[i][None, None, :], self.polygon_vertices)[0, 0])
            edge_dist = distance_to_polygon_edge(self.pos[i], self.polygon_vertices)
            if (not inside) or (edge_dist < min_edge_keep):
                # Snap back toward centroid until safe
                for _ in range(10):
                    nudge = (centroid - self.pos[i])
                    nudge_len = np.linalg.norm(nudge)
                    if nudge_len < 1e-6:
                        break
                    self.pos[i] += (nudge / nudge_len) * 0.2
                    inside = bool(points_in_polygon(self.pos[i][None, None, :], self.polygon_vertices)[0, 0])
                    edge_dist = distance_to_polygon_edge(self.pos[i], self.polygon_vertices)
                    if inside and edge_dist >= min_edge_keep:
                        break
                else:
                    # Final fallback
                    self.pos[i] = self.prev_pos[i].copy()
                hard_wall[i] = 1.0

        # ── SAFETY TRACKING: count violations for hard gate ──
        # Drone-drone: any pair closer than drone_safety_radius
        if len(active_idxs) >= 2:
            pos_a = self.pos[active_idxs]
            diff_s = pos_a[:, None, :] - pos_a[None, :, :]
            dists_s = np.sqrt((diff_s ** 2).sum(axis=-1))
            np.fill_diagonal(dists_s, 999.0)
            n_violations = int(np.sum(dists_s < self.cfg.drone_safety_radius)) // 2  # each pair counted twice
            if n_violations > 0:
                self._episode_collision_count += n_violations
                self._safety_violated = True
        # Wall: hard_wall flag means physics intervened
        wall_violations = int(np.sum(hard_wall[active_idxs] > 0.5))
        if wall_violations > 0:
            self._episode_wall_collision_count += wall_violations
            self._safety_violated = True

        self.step_count += 1
        self._maybe_drop_agents()

        # ── record position history ──
        self._pos_history[:, self._pos_hist_ptr % self._displacement_window] = self.pos.copy()
        self._pos_hist_ptr += 1

        # ── update trajectory heatmap (shared via communication) ──
        self._trajectory_heatmap *= self.cfg.trajectory_heatmap_decay
        self._drone_trajectory_maps *= self.cfg.trajectory_heatmap_decay
        ix_t, iy_t = self._world_to_grid(self.pos)
        for i in np.where(self.active_mask > 0.5)[0]:
            # Mark a small disk (radius=2 cells = 1m) so trajectory is visible after downsampling
            self._draw_disk(self._trajectory_heatmap, ix_t[i], iy_t[i], 2, 1.0)
            self._drone_trajectory_maps[i, iy_t[i], ix_t[i]] = 1.0

        # Snapshot before coverage update — needed by _new_cells_bonus
        self._ever_visited_before = self.ever_visited.copy()

        cov = self._update_coverage()

        overlap = self._overlap_ratio()
        collision, min_dist = self._collision_penalty()
        wall = self._wall_penalty(hard_wall)
        stagnation = self._stagnation_penalty()
        circling = self._circling_penalty()
        control = np.mean(np.abs(clipped - self.prev_actions), axis=1).astype(np.float32)
        frontier = self._frontier_progress()
        team_reward = self._team_reward(cov)
        displacement = self._displacement_bonus()
        revisit = self._revisit_penalty()
        heading_spread = self._heading_spread_bonus()
        spread = self._spread_bonus()
        new_cells = self._new_cells_bonus()
        frontier_align = self._frontier_align_reward()
        local_staleness = self._local_staleness_penalty()
        scan_gap = self._scan_gap_penalty()
        corner_bonus = self._corner_scan_bonus()
        frontier_dist_pen = self._frontier_distance_penalty()
        fairness_pen = self._fairness_penalty()
        late_hunt_bonus = self._late_game_hunting_bonus()
        local_resp = self._local_responsibility_bonus()
        hole_left = self._hole_left_behind_penalty()
        maint_need_bonus = self._maintenance_need_reduction_bonus()

        # ── Sustained turn penalty: penalize turning same direction >10 steps ──
        sustained_turn = np.zeros((self.max_agents,), dtype=np.float32)
        for i in np.where(self.active_mask > 0.5)[0]:
            streak = abs(int(self._turn_sign_streak[i]))
            if streak > 10:  # ~1 second of continuous same-direction turning
                sustained_turn[i] = min(1.0, (streak - 10) / 20.0)  # ramps 0→1 over 10→30 steps

        rewards = np.zeros((self.max_agents,), dtype=np.float32)
        rewards += team_reward
        rewards += self.cfg.w_frontier_progress * frontier * self.active_mask
        rewards += self.cfg.w_alive * self.active_mask
        rewards += self.cfg.w_heading_spread * heading_spread
        rewards += self.cfg.w_displacement * displacement
        rewards += self.cfg.w_spread * spread
        rewards += self.cfg.w_new_cells * new_cells
        rewards += self.cfg.w_frontier_align * frontier_align
        rewards += self.cfg.w_corner_bonus * corner_bonus
        rewards += self.cfg.w_late_hunt * late_hunt_bonus
        rewards += self.cfg.w_local_responsibility * local_resp
        rewards -= self.cfg.w_hole_left_behind * hole_left
        rewards -= self.cfg.w_overlap * overlap
        rewards -= self.cfg.w_collision * collision
        rewards -= self.cfg.w_wall * wall
        rewards -= self.cfg.w_stagnation * stagnation
        rewards -= self.cfg.w_circling * circling
        rewards -= self.cfg.w_control * control
        rewards -= self.cfg.w_revisit * revisit
        rewards -= self.cfg.w_local_staleness * local_staleness
        rewards -= self.cfg.w_scan_gap * scan_gap
        rewards -= self.cfg.w_frontier_dist * frontier_dist_pen
        rewards -= self.cfg.w_fairness * fairness_pen
        rewards -= self.cfg.w_sustained_turn * sustained_turn
        rewards *= self.active_mask
        self.prev_actions = clipped.copy()

        obs = self.get_actor_observation()
        info = self._build_info(rewards)
        info.update({
            'team_reward': float(team_reward),
            'overlap_ratio_mean': float(np.sum(overlap) / max(1.0, np.sum(self.active_mask))),
            'collision_penalty_mean': float(np.sum(collision) / max(1.0, np.sum(self.active_mask))),
            'wall_penalty_mean': float(np.sum(wall) / max(1.0, np.sum(self.active_mask))),
            'displacement_bonus_mean': float(np.sum(displacement) / max(1.0, np.sum(self.active_mask))),
            'revisit_penalty_mean': float(np.sum(revisit) / max(1.0, np.sum(self.active_mask))),
            'heading_spread_mean': float(np.sum(heading_spread) / max(1.0, np.sum(self.active_mask))),
            'spread_bonus_mean': float(np.sum(spread) / max(1.0, np.sum(self.active_mask))),
            'circling_penalty_mean': float(np.sum(circling) / max(1.0, np.sum(self.active_mask))),
            'min_inter_drone_dist': float(min_dist),
            'active_agents': float(np.sum(self.active_mask)),
            'dropped_agents': float(self.total_dropped_agents),
            'scan_efficiency_score': self._scan_efficiency_score(),
            'polygon_area_m2': float(polygon_area(self.polygon_vertices)),
            # ── reward breakdown (sum over all drones) ──
            'rw_team': float(team_reward * np.sum(self.active_mask)),
            'rw_frontier': float(np.sum(self.cfg.w_frontier_progress * frontier * self.active_mask)),
            'rw_heading_spread': float(np.sum(self.cfg.w_heading_spread * heading_spread)),
            'rw_displacement': float(np.sum(self.cfg.w_displacement * displacement)),
            'rw_spread': float(np.sum(self.cfg.w_spread * spread)),
            'rw_overlap': float(-np.sum(self.cfg.w_overlap * overlap)),
            'rw_collision': float(-np.sum(self.cfg.w_collision * collision)),
            'rw_wall': float(-np.sum(self.cfg.w_wall * wall)),
            'rw_stagnation': float(-np.sum(self.cfg.w_stagnation * stagnation)),
            'rw_circling': float(-np.sum(self.cfg.w_circling * circling)),
            'rw_control': float(-np.sum(self.cfg.w_control * control)),
            'rw_revisit': float(-np.sum(self.cfg.w_revisit * revisit)),
            'rw_new_cells': float(np.sum(self.cfg.w_new_cells * new_cells)),
            'rw_frontier_align': float(np.sum(self.cfg.w_frontier_align * frontier_align)),
            'rw_corner_bonus': float(np.sum(self.cfg.w_corner_bonus * corner_bonus)),
            'rw_local_staleness': float(-np.sum(self.cfg.w_local_staleness * local_staleness)),
            'rw_scan_gap': float(-np.sum(self.cfg.w_scan_gap * scan_gap)),
            'rw_frontier_dist': float(-np.sum(self.cfg.w_frontier_dist * frontier_dist_pen)),
            'rw_fairness': float(-np.sum(self.cfg.w_fairness * fairness_pen)),
            'rw_sustained_turn': float(-np.sum(self.cfg.w_sustained_turn * sustained_turn)),
            'rw_late_hunt': float(np.sum(self.cfg.w_late_hunt * late_hunt_bonus)),
            'rw_local_resp': float(np.sum(self.cfg.w_local_responsibility * local_resp)),
            'rw_hole_left': float(-np.sum(self.cfg.w_hole_left_behind * hole_left)),
            'rw_maint_need': float(np.sum(self.cfg.w_maint_need_reduction * maint_need_bonus)),
            # ── SAFETY hard gate ──
            'safety_violated': self._safety_violated,
            'episode_collision_count': self._episode_collision_count,
            'episode_wall_collision_count': self._episode_wall_collision_count,
        })
        return obs, rewards, False, self.step_count >= self.max_steps, info

    def _maybe_drop_agents(self):
        if not self.cfg.allow_mid_episode_failures:
            return
        active_idxs = np.where(self.active_mask > 0.5)[0]
        if len(active_idxs) <= 1:
            return
        p = self.cfg.failure_probability_per_second * self.dt
        dropped = 0
        for idx in active_idxs:
            if len(np.where(self.active_mask > 0.5)[0]) <= 1:
                break
            if self._rng.random() < p:
                self.active_mask[idx] = 0.0
                self.vel[idx] = 0.0
                dropped += 1
        self.total_dropped_agents += dropped

    def _scan_masks(self):
        if self._cached_scan_step == self.step_count and self._cached_scan_masks is not None:
            return self._cached_scan_masks
        masks = np.zeros((self.max_agents, self.grid_h, self.grid_w), dtype=bool)
        active_idxs = np.where(self.active_mask > 0.5)[0]
        if len(active_idxs) == 0:
            self._cached_scan_masks = masks
            self._cached_scan_step = self.step_count
            return masks
        half_fov = math.radians(self.cfg.scan_fov_deg) / 2.0
        # Batch: (n_active, grid_h, grid_w, 2)
        pos_active = self.pos[active_idxs]  # (n, 2)
        rel = self.grid_points[None, :, :, :] - pos_active[:, None, None, :]  # (n, H, W, 2)
        dist = np.sqrt(rel[..., 0] ** 2 + rel[..., 1] ** 2)  # (n, H, W)
        angle = np.arctan2(rel[..., 1], rel[..., 0]) - self.heading[active_idxs, None, None]
        angle = (angle + np.pi) % (2.0 * np.pi) - np.pi

        # FOV mask — always use the configured FOV (no corner override)
        fov_mask = np.abs(angle) <= half_fov  # (n, H, W)

        in_range = (dist <= self.cfg.scan_range_m) & fov_mask & self.active_area_mask[None, :, :]
        masks[active_idxs] = in_range
        self._cached_scan_masks = masks
        self._cached_scan_step = self.step_count
        return masks

    def _update_coverage(self):
        prev_ever = self.last_ever_seen_fraction
        prev_maintained = self.last_maintained_fraction
        self.coverage_age[self.active_area_mask] = np.minimum(self.coverage_age[self.active_area_mask] + self.dt, self.cfg.decay_reset_seconds)
        self.coverage_value[self.active_area_mask] = np.clip(1.0 - self.coverage_age[self.active_area_mask] / self.cfg.decay_reset_seconds, 0.0, 1.0)
        masks = self._scan_masks()
        visible = np.any(masks, axis=0) & self.active_area_mask
        owners = -np.ones((self.grid_h, self.grid_w), dtype=np.int16)
        coords = np.argwhere(visible)
        active_idxs = np.where(self.active_mask > 0.5)[0]
        if len(coords) > 0 and len(active_idxs) > 0:
            pts = self.grid_points[coords[:, 0], coords[:, 1]]
            dists = np.linalg.norm(pts[:, None, :] - self.pos[active_idxs][None, :, :], axis=-1)
            owner_local = np.argmin(dists, axis=1)
            owners[coords[:, 0], coords[:, 1]] = active_idxs[owner_local]
        total_active_cells = max(1, int(np.sum(self.active_area_mask)))
        owned = owners >= 0
        # Track per-drone new discoveries this step
        new_cells_mask = (self.ever_visited == 0) & owned
        self._step_discoveries = np.zeros((self.max_agents,), dtype=np.int32)
        if np.any(new_cells_mask):
            new_coords = np.argwhere(new_cells_mask)
            for r, c in new_coords:
                drone_id = owners[r, c]
                if drone_id >= 0:
                    self._drone_discoveries[drone_id] += 1
                    self._step_discoveries[drone_id] += 1
        first_visit_frac = float(np.sum(new_cells_mask)) / total_active_cells if np.any(owned) else 0.0
        refresh_gain_frac = float(np.sum((1.0 - self.coverage_value) * owned)) / total_active_cells if np.any(owned) else 0.0

        # ── Maintenance NEED computation (before applying new scans) ──
        # Need = f(time_since_last_scan) × geometry_weight × (1 - diminishing_returns)
        # Only for cells that have been visited at least once (maintenance doesn't apply to unexplored)
        visited_mask = (self.ever_visited > 0) & self.active_area_mask

        # Time-based need: exponential growth since last scan (saturates at 1.0)
        # need_time(t) = 1 - exp(-age / tau)
        tau = max(1e-3, self.cfg.maint_need_decay_tau)
        time_need = np.where(
            visited_mask,
            1.0 - np.exp(-self.coverage_age / tau),
            0.0
        ).astype(np.float32)

        # Diminishing returns factor: recent scans reduce the value of rescanning
        # Factor in [0, 1]: 1.0 = no recent scans (full value), → 0 with many recent scans
        # _maint_recent_scans accumulates and decays; factor = exp(-accumulator)
        diminishing_factor = np.exp(-self._maint_recent_scans).astype(np.float32)

        # Full maintenance need = time_need × geometry × diminishing × visited
        self._maint_need_map = (time_need * self._maint_geometry_weight * diminishing_factor).astype(np.float32)
        self._maint_need_map[~self.active_area_mask] = 0.0

        # Snapshot BEFORE applying scans (for gain = need_before - need_after)
        self._maint_need_before = self._maint_need_map.copy()

        # ── Decay the recent-scan accumulator (exponential decay each step) ──
        half_life_steps = max(1.0, self.cfg.maint_diminishing_half_life / self.dt)
        decay_rate = math.exp(-math.log(2.0) / half_life_steps)
        self._maint_recent_scans *= decay_rate

        # ── Record into fairness sliding window ──
        widx = self._recent_disc_ptr % self._fairness_window
        self._recent_discoveries[:, widx] = self._step_discoveries
        self._recent_disc_ptr += 1

        if np.any(owned):
            self.coverage_age[owned] = 0.0
            self.coverage_value[owned] = 1.0
            self.ever_visited[owned] = 1
            self.last_owner[owned] = owners[owned]

            # ── Increment recent-scan accumulator for scanned cells ──
            # Each scan adds 1.0 to the accumulator → diminishing returns for rapid rescans
            self._maint_recent_scans[owned] += 1.0

        # ── Compute need AFTER scan (need drops to ~0 for freshly scanned cells) ──
        visited_after = (self.ever_visited > 0) & self.active_area_mask
        time_need_after = np.where(
            visited_after,
            1.0 - np.exp(-self.coverage_age / max(1e-3, self.cfg.maint_need_decay_tau)),
            0.0
        ).astype(np.float32)
        diminishing_after = np.exp(-self._maint_recent_scans).astype(np.float32)
        need_after = (time_need_after * self._maint_geometry_weight * diminishing_after).astype(np.float32)
        need_after[~self.active_area_mask] = 0.0

        # ── Per-drone maintenance gain = sum of need reduction for cells this drone scanned ──
        # gain = max(0, need_before - need_after) for each cell owned by this drone
        need_reduction = np.maximum(0.0, self._maint_need_before - need_after)
        self._step_maint_gain = np.zeros((self.max_agents,), dtype=np.float32)
        if np.any(owned):
            for drone_idx in np.unique(owners[owned]):
                if drone_idx < 0:
                    continue
                drone_cells = owned & (owners == drone_idx)
                self._step_maint_gain[drone_idx] = float(np.sum(need_reduction[drone_cells]))
            self._drone_maint_gains += self._step_maint_gain

        # Compute team-level maintenance need reduction (normalized)
        total_need_before = float(np.sum(self._maint_need_before))
        total_need_after = float(np.sum(need_after))
        maint_need_reduction_frac = max(0.0, total_need_before - total_need_after) / max(1.0, total_need_before) if total_need_before > 0.1 else 0.0
        active_values = self.coverage_value[self.active_area_mask]
        ever_seen_fraction = float(np.mean(self.ever_visited[self.active_area_mask]))
        maintained_fraction = float(np.mean(active_values >= self.cfg.maintained_threshold))
        self.last_ever_seen_fraction = ever_seen_fraction
        self.last_maintained_fraction = maintained_fraction
        t = self.step_count * self.dt
        if self.time_to_50 < 0 and ever_seen_fraction >= 0.50:
            self.time_to_50 = t
        if self.time_to_80 < 0 and ever_seen_fraction >= 0.80:
            self.time_to_80 = t
        if self.time_to_95 < 0 and ever_seen_fraction >= 0.95:
            self.time_to_95 = t
        return {
            'first_visit_frac': first_visit_frac,
            'refresh_gain_frac': refresh_gain_frac,
            'ever_seen_gain': max(0.0, ever_seen_fraction - prev_ever),
            'maintained_gain': maintained_fraction - prev_maintained,
            'ever_seen_fraction': ever_seen_fraction,
            'maintained_fraction': maintained_fraction,
            'uncovered_fraction': 1.0 - ever_seen_fraction,
            'stale_fraction': float(np.mean(active_values < self.cfg.maintained_threshold)),
            'maint_need_reduction': maint_need_reduction_frac,
        }

    def _team_reward(self, cov: dict[str, float]) -> float:
        es = cov['ever_seen_fraction']

        # SMOOTH PHASE BLENDING (replaces hard Phase1/Phase2 binary gate):
        # Instead of a hard gate at 100%, blend exploration and maintenance.
        # - Below 90%: pure exploration (maintenance_weight = 0)
        # - 90%->100%: linearly blend in maintenance (0->1)
        # - At/above 100%: pure maintenance
        # This ensures maintenance rewards are available before 100% is reached,
        # preventing the "never reach Phase 2" deadlock.
        if es < 0.90:
            maint_weight = 0.0
        elif es >= 1.0 or self._milestone_hit.get(100, False):
            maint_weight = 1.0
        else:
            maint_weight = (es - 0.90) / 0.10  # 0 at 90%, 1 at 100%
        explore_weight = 1.0 - maint_weight

        # ── Escalating first-visit reward ──
        # Finding the last 5% is worth 12x more than the first 5%
        escalation = 1.0 + 11.0 * (es ** 4)  # 1.0 at 0%, ~1.7 at 50%, 12.0 at 100%

        r = 0.0

        # ── Exploration component (always present, fades at high coverage) ──
        r += explore_weight * self.cfg.w_first_visit * cov['first_visit_frac'] * escalation
        r += explore_weight * self.cfg.w_ever_seen_gain * cov['ever_seen_gain'] * escalation
        # Per-step penalty for remaining uncovered cells (grows aggressively with coverage)
        uncovered_penalty_mult = 1.0 + 4.0 * (es ** 2) + 20.0 * max(0.0, es - 0.95) ** 2
        r -= explore_weight * self.cfg.w_team_uncovered_penalty * cov['uncovered_fraction'] * uncovered_penalty_mult

        # ── Maintenance component (blends in starting at 90%) ──
        r += maint_weight * self.cfg.w_refresh_gain * cov['refresh_gain_frac']
        r += maint_weight * self.cfg.w_maintained_gain * cov['maintained_gain']
        r -= maint_weight * self.cfg.w_team_stale_penalty * cov['stale_fraction']
        # Still reward first visits during maintenance (in case some cells were missed)
        r += maint_weight * self.cfg.w_first_visit * cov['first_visit_frac'] * 2.0

        # ── One-time milestone bonuses ──
        milestones = [
            (50, self.cfg.w_milestone_50),
            (80, self.cfg.w_milestone_80),
            (90, self.cfg.w_milestone_90),
            (95, self.cfg.w_milestone_95),
            (98, self.cfg.w_milestone_98),
            (100, self.cfg.w_milestone_100),
        ]
        for pct, bonus in milestones:
            if es >= pct / 100.0 and not self._milestone_hit[pct]:
                self._milestone_hit[pct] = True
                r += bonus

        return float(r)

    def _overlap_ratio(self) -> np.ndarray:
        masks = self._scan_masks()
        out = np.zeros((self.max_agents,), dtype=np.float32)
        active_idxs = np.where(self.active_mask > 0.5)[0]
        n = len(active_idxs)
        if n < 2:
            return out
        # Flatten masks to (n, H*W) for fast pairwise ops
        flat = masks[active_idxs].reshape(n, -1)
        # Pairwise intersection and union counts: (n, n)
        inter = flat.astype(np.float32) @ flat.astype(np.float32).T  # dot = intersection count
        counts = flat.sum(axis=1, dtype=np.float32)  # per-agent cell count
        union = counts[:, None] + counts[None, :] - inter
        union = np.maximum(union, 1.0)
        iou = inter / union
        np.fill_diagonal(iou, 0.0)
        # Mean IoU with all other agents
        mean_iou = iou.sum(axis=1) / max(1, n - 1)
        out[active_idxs] = mean_iou.astype(np.float32)
        return out

    def _collision_penalty(self):
        """SAFETY CRITICAL: penalize drones that are too close to each other.
        Returns penalty per drone and minimum inter-drone distance."""
        pen = np.zeros((self.max_agents,), dtype=np.float32)
        active_idxs = np.where(self.active_mask > 0.5)[0]
        n = len(active_idxs)
        min_dist = 999.0
        if n < 2:
            return pen * self.active_mask, min_dist
        r = self.cfg.drone_safety_radius  # 1.5m center-to-center
        pos_a = self.pos[active_idxs]
        diff = pos_a[:, None, :] - pos_a[None, :, :]
        dists = np.sqrt((diff ** 2).sum(axis=-1))
        np.fill_diagonal(dists, 999.0)
        min_dist = float(dists.min())
        # DANGER zone: d < r (violating safety radius) — MASSIVE penalty
        mask_hard = dists < r
        overlap_ratio = np.clip((r - dists) / max(1e-6, r), 0, 1)
        p_hard = 10.0 + 20.0 * (overlap_ratio ** 2)  # 10 at boundary, 30 at zero distance
        # WARNING zone: r <= d < 2.5r (getting too close) — escalating penalty
        mask_warn = (dists >= r) & (dists < 2.5 * r)
        p_warn = 3.0 * ((2.5 * r - dists) / (1.5 * r)) ** 2  # quadratic ramp
        # CAUTION zone: 2.5r <= d < 3.5r — gentle nudge
        mask_caution = (dists >= 2.5 * r) & (dists < 3.5 * r)
        p_caution = 0.5 * ((3.5 * r - dists) / r)
        total = (p_hard * mask_hard + p_warn * mask_warn + p_caution * mask_caution)
        pen_active = total.sum(axis=1).astype(np.float32)
        pen[active_idxs] = pen_active
        return pen * self.active_mask, min_dist

    def _wall_penalty(self, hard: np.ndarray) -> np.ndarray:
        """Wall penalty. SAFETY CRITICAL — drones MUST NOT touch walls.
        Penalty ramps from 0 at safe distance to 10+ at wall contact."""
        pen = hard.astype(np.float32) * 5.0  # 5.0 if physics forced a wall correction
        margin = self.cfg.wall_safety_margin  # 1.0m — DANGER ZONE
        soft_margin = margin + 1.5  # 2.5m — caution zone starts here
        for i in np.where(self.active_mask > 0.5)[0]:
            d = distance_to_polygon_edge(self.pos[i], self.polygon_vertices)
            if d < margin:
                ratio = (margin - d) / margin  # 0→1 as d goes margin→0
                pen[i] += 5.0 + 10.0 * (ratio ** 2)  # 5 at margin, 15 at wall
            elif d < soft_margin:
                ratio = (soft_margin - d) / (soft_margin - margin)  # 0→1
                pen[i] += 2.0 * (ratio ** 2)  # quadratic ramp, 0 at soft_margin, 2 at margin
        return pen * self.active_mask

    def _stagnation_penalty(self) -> np.ndarray:
        moved = np.linalg.norm(self.pos - self.prev_pos, axis=1)
        return np.clip((0.12 - moved) / 0.12, 0.0, 1.0).astype(np.float32) * self.active_mask

    def _circling_penalty(self) -> np.ndarray:
        """Penalise drones whose net displacement is low relative to distance
        travelled — i.e. they are looping back on themselves.

        Uses THREE windows (short/medium/long) to catch circles of all radii.
        Also detects excessive cumulative turning (high heading change over
        the window = spinning in place or tight circles).
        """
        pen = np.zeros((self.max_agents,), dtype=np.float32)
        n_filled = min(self._pos_hist_ptr, self._displacement_window)
        if n_filled < 6:
            return pen
        oldest_idx = (self._pos_hist_ptr - n_filled) % self._displacement_window

        # Three windows to catch tight, medium, and wide loops
        n_short = max(6, n_filled // 4)     # ~1.5s — catches tight loops
        n_mid = max(6, n_filled // 2)       # ~2.5s — catches medium loops
        oldest_short = (self._pos_hist_ptr - n_short) % self._displacement_window
        oldest_mid = (self._pos_hist_ptr - n_mid) % self._displacement_window

        for i in np.where(self.active_mask > 0.5)[0]:
            max_pen = 0.0
            for (n_win, old_idx) in [(n_filled, oldest_idx), (n_mid, oldest_mid), (n_short, oldest_short)]:
                old_pos = self._pos_history[i, old_idx]
                net_disp = float(np.linalg.norm(self.pos[i] - old_pos))
                path_len = 0.0
                for k in range(1, n_win):
                    idx_prev = (old_idx + k - 1) % self._displacement_window
                    idx_curr = (old_idx + k) % self._displacement_window
                    path_len += float(np.linalg.norm(
                        self._pos_history[i, idx_curr] - self._pos_history[i, idx_prev]))
                if path_len > 0.3:  # lowered from 0.8 to catch slow circles
                    straightness = net_disp / path_len
                    # Penalize if straightness < 0.5 (was 0.6 — tighter threshold)
                    # Amplification 5.0 (was 3.0 — much stronger feedback)
                    p = max(0.0, 0.5 - straightness) * 5.0
                    max_pen = max(max_pen, p)

            # ── Heading-change detection ──
            # If the drone has turned more than 2π radians total over the window,
            # it's likely circling even if straightness looks okay (e.g., figure-8).
            if n_filled >= 10:
                heading_changes = 0.0
                for k in range(1, min(n_filled, 30)):
                    idx_prev = (self._pos_hist_ptr - k) % self._displacement_window
                    idx_curr = (self._pos_hist_ptr - k + 1) % self._displacement_window
                    # Use position-based heading change as proxy
                    dp = self._pos_history[i, idx_curr] - self._pos_history[i, idx_prev]
                    dp_norm = np.linalg.norm(dp)
                    if dp_norm > 0.01:
                        heading_changes += 1.0  # count steps with movement
                # Combine with straightness: if lots of movement but low displacement
                if heading_changes > 5:
                    recent_disp = float(np.linalg.norm(
                        self.pos[i] - self._pos_history[i, (self._pos_hist_ptr - min(n_filled, 30)) % self._displacement_window]))
                    recent_path = heading_changes * 0.1  # rough step distance
                    if recent_path > 0.3 and recent_disp < recent_path * 0.4:
                        turn_pen = (1.0 - recent_disp / recent_path) * 2.0
                        max_pen = max(max_pen, turn_pen)

            pen[i] = max_pen
        return pen * self.active_mask

    def _displacement_bonus(self) -> np.ndarray:
        """Reward drones proportionally to how far they moved over the recent
        rolling window.  This is a *positive* incentive for covering ground —
        the policy discovers on its own that looping wastes displacement."""
        bonus = np.zeros((self.max_agents,), dtype=np.float32)
        n_filled = min(self._pos_hist_ptr, self._displacement_window)
        if n_filled < 2:
            return bonus
        oldest_idx = (self._pos_hist_ptr - n_filled) % self._displacement_window
        for i in np.where(self.active_mask > 0.5)[0]:
            old_pos = self._pos_history[i, oldest_idx]
            disp = float(np.linalg.norm(self.pos[i] - old_pos))
            # normalise by the maximum possible displacement in the window
            max_disp = self.cfg.max_speed * n_filled * self.dt
            bonus[i] = min(disp / max(1e-6, max_disp), 1.0)
        return bonus * self.active_mask

    def _revisit_penalty(self) -> np.ndarray:
        """Penalise scanning cells that are already visited.

        Uses smooth phase blending (consistent with _team_reward):
        - Below 90% coverage: HARSH penalty (threshold=0.3, 2.5x multiplier)
        - 90-100%: linearly blend toward lenient
        - Above 100% / maintenance: Lenient (threshold=0.7, 1.0x multiplier)
        """
        pen = np.zeros((self.max_agents,), dtype=np.float32)
        masks = self._scan_masks()
        es = self.last_ever_seen_fraction

        # Smooth blending instead of hard phase gate
        if es < 0.90:
            threshold = 0.3
            multiplier = 2.5
        elif es >= 1.0 or self._milestone_hit.get(100, False):
            threshold = 0.7
            multiplier = 1.0
        else:
            blend = (es - 0.90) / 0.10  # 0 at 90%, 1 at 100%
            threshold = 0.3 + 0.4 * blend  # 0.3 → 0.7
            multiplier = 2.5 - 1.5 * blend  # 2.5 → 1.0

        for i in np.where(self.active_mask > 0.5)[0]:
            scan_cells = masks[i] & self.active_area_mask
            total = int(np.sum(scan_cells))
            if total < 1:
                continue
            already_seen = int(np.sum(scan_cells & (self.ever_visited > 0)))
            frac_revisit = already_seen / total
            if frac_revisit > threshold:
                pen[i] = ((frac_revisit - threshold) / (1.0 - threshold)) * multiplier
        return pen * self.active_mask

    def _heading_spread_bonus(self) -> np.ndarray:
        """Reward drones for spreading out in heading — encourages covering
        different directions instead of all going the same way."""
        bonus = np.zeros((self.max_agents,), dtype=np.float32)
        active_idxs = np.where(self.active_mask > 0.5)[0]
        if len(active_idxs) < 2:
            return bonus
        headings_active = self.heading[active_idxs]
        # compute mean angular distance to other drones' headings
        for idx_pos, i in enumerate(active_idxs):
            diffs = np.abs(headings_active - self.heading[i])
            diffs = np.minimum(diffs, 2 * np.pi - diffs)
            # exclude self
            mask = np.ones(len(active_idxs), dtype=bool)
            mask[idx_pos] = False
            mean_diff = float(np.mean(diffs[mask]))
            # normalise: max angular diff is π, bonus peaks at π
            bonus[i] = mean_diff / np.pi
        return bonus * self.active_mask

    def _spread_bonus(self) -> np.ndarray:
        """Reward each drone for being far from all other active drones.

        Returns a per-agent bonus in [0, 1] based on the minimum distance to
        any other active drone, normalised by the ideal spacing (polygon area
        divided evenly among drones).
        """
        bonus = np.zeros((self.max_agents,), dtype=np.float32)
        active_idxs = np.where(self.active_mask > 0.5)[0]
        n = len(active_idxs)
        if n < 2:
            return bonus
        # ideal spacing: if drones tile the polygon evenly
        area = float(polygon_area(self.polygon_vertices))
        ideal_dist = math.sqrt(area / max(n, 1))
        pos_a = self.pos[active_idxs]
        diff = pos_a[:, None, :] - pos_a[None, :, :]
        dists = np.sqrt((diff ** 2).sum(axis=-1))
        np.fill_diagonal(dists, 1e9)
        min_d = dists.min(axis=1)
        bonus[active_idxs] = np.minimum(min_d / max(1e-6, ideal_dist), 1.0).astype(np.float32)
        return bonus * self.active_mask

    def _new_cells_bonus(self) -> np.ndarray:
        """Per-drone reward: fraction of this drone's scan circle that was NEW
        (never-seen before this step).  Range [0, 1].
        0 = scanning entirely old territory, 1 = 100% fresh cells."""
        bonus = np.zeros((self.max_agents,), dtype=np.float32)
        masks = self._scan_masks()
        was_unseen = (self._ever_visited_before == 0) & self.active_area_mask
        for i in np.where(self.active_mask > 0.5)[0]:
            scan_total = int(np.sum(masks[i] & self.active_area_mask))
            if scan_total < 1:
                continue
            new_in_scan = int(np.sum(masks[i] & was_unseen))
            bonus[i] = new_in_scan / scan_total  # 0-1: fraction of scan that was new
        return bonus * self.active_mask

    def _frontier_align_reward(self) -> np.ndarray:
        """Reward each drone for having its velocity vector aligned with the
        direction toward its frontier waypoint.  This directly incentivizes
        the drone to MOVE TOWARD unseen areas rather than circling."""
        bonus = np.zeros((self.max_agents,), dtype=np.float32)
        frontier_wp = self._compute_frontier_waypoints()
        for i in np.where(self.active_mask > 0.5)[0]:
            wp_dir = frontier_wp[i, :2]
            wp_dist = frontier_wp[i, 2]
            if wp_dist < 1e-3:
                continue  # no frontier waypoint
            vel_norm = np.linalg.norm(self.vel[i])
            if vel_norm < 1e-6:
                continue
            vel_dir = self.vel[i] / vel_norm
            # Dot product: +1 = heading straight toward frontier, -1 = away
            alignment = float(np.dot(vel_dir, wp_dir))
            # Only reward positive alignment, scaled by speed fraction
            speed_frac = vel_norm / self.cfg.max_speed
            bonus[i] = max(0.0, alignment) * speed_frac
        return bonus * self.active_mask

    def _local_staleness_penalty(self) -> np.ndarray:
        """Penalize drones lingering in already-covered areas WITH NO uncovered
        cells nearby.

        CRITICAL FIX: If there are ANY uncovered active cells within the local
        patch, the penalty is SUPPRESSED — the drone should stay and cover them!
        This prevents the corner problem where drones were pushed away from
        mostly-covered areas that still had a few critical uncovered cells.

        Phase 1 (coverage): Fire at 60% local saturation (if no uncovered nearby).
        Phase 2 (maintenance): Fire at 90%.
        """
        pen = np.zeros((self.max_agents,), dtype=np.float32)
        phase1_active = not self._milestone_hit.get(100, False)
        global_cov = self.last_ever_seen_fraction

        if phase1_active:
            threshold = 0.60
            multiplier = 2.0 + 3.0 * (global_cov ** 2)
        else:
            threshold = 0.90
            multiplier = 1.0

        for i in np.where(self.active_mask > 0.5)[0]:
            scan_r_cells = int(self.cfg.scan_range_m / self.cfg.grid_resolution_m)
            ix, iy = self._world_to_grid(self.pos[i:i + 1])
            ix, iy = int(ix[0]), int(iy[0])
            y_lo = max(0, iy - scan_r_cells)
            y_hi = min(self.grid_h, iy + scan_r_cells + 1)
            x_lo = max(0, ix - scan_r_cells)
            x_hi = min(self.grid_w, ix + scan_r_cells + 1)
            active_patch = self.active_area_mask[y_lo:y_hi, x_lo:x_hi]
            total = np.sum(active_patch)
            if total < 1:
                continue
            ever_patch = self.ever_visited[y_lo:y_hi, x_lo:x_hi]

            # KEY FIX: If there are uncovered cells nearby, do NOT penalize!
            # The drone should stay to cover them.
            uncovered_nearby = int(np.sum((ever_patch == 0) & active_patch))
            if uncovered_nearby > 0:
                continue  # suppress penalty — there's work to do here

            frac_visited = float(np.sum(ever_patch[active_patch])) / float(total)
            if frac_visited > threshold:
                pen[i] = ((frac_visited - threshold) / (1.0 - threshold)) * multiplier
        return pen * self.active_mask

    def _scan_gap_penalty(self) -> np.ndarray:
        """Penalize drones that are moving FAST while leaving unseen cells
        within their scan range BEHIND them. This forces thorough local coverage
        before moving to new areas — preventing the 'holes' problem.

        Only penalizes if:
        1. Drone is moving (speed > 30% of max)
        2. There are unseen active cells within scan_range behind/beside the drone
        Returns [0, 1] per drone.
        """
        pen = np.zeros((self.max_agents,), dtype=np.float32)
        scan_r_cells = int(self.cfg.scan_range_m / self.cfg.grid_resolution_m)

        for i in np.where(self.active_mask > 0.5)[0]:
            speed = np.linalg.norm(self.vel[i])
            speed_frac = speed / self.cfg.max_speed
            if speed_frac < 0.3:
                continue  # not moving fast enough to leave gaps

            ix, iy = self._world_to_grid(self.pos[i:i + 1])
            ix, iy = int(ix[0]), int(iy[0])
            y_lo = max(0, iy - scan_r_cells)
            y_hi = min(self.grid_h, iy + scan_r_cells + 1)
            x_lo = max(0, ix - scan_r_cells)
            x_hi = min(self.grid_w, ix + scan_r_cells + 1)
            active_patch = self.active_area_mask[y_lo:y_hi, x_lo:x_hi]
            ever_patch = self.ever_visited[y_lo:y_hi, x_lo:x_hi]
            total_active = np.sum(active_patch)
            if total_active < 1:
                continue
            unseen_nearby = float(np.sum((ever_patch == 0) & active_patch)) / float(total_active)
            # Penalize proportionally to unseen fraction AND speed
            # Fast + lots of unseen nearby = high penalty (leaving gaps!)
            if unseen_nearby > 0.1:  # more than 10% unseen in scan range
                pen[i] = unseen_nearby * speed_frac
        return pen * self.active_mask

    def _frontier_distance_penalty(self) -> np.ndarray:
        """Penalize each drone proportionally to its distance from the nearest
        uncovered cell. FULL RESOLUTION — no subsampling.

        Above 97%, this becomes the DOMINANT signal in the reward landscape.
        The penalty grows EXPONENTIALLY to make uncovered cells irresistible attractors.

        Activation: 50% coverage.
        At >97%: penalty is 15× base. At >99%: penalty is 30× base.
        """
        pen = np.zeros((self.max_agents,), dtype=np.float32)
        frontier = self.active_area_mask & (self.ever_visited == 0)
        if not np.any(frontier):
            return pen  # everything covered — no penalty

        es = self.last_ever_seen_fraction
        if es < 0.50:
            return pen

        # EXPONENTIAL late-game scaling — uncovered cells become irresistible
        if es < 0.90:
            late_mult = 0.3 + 1.0 * ((es - 0.50) / 0.40)
        elif es < 0.97:
            late_mult = 1.3 + 6.0 * ((es - 0.90) / 0.07)
        elif es < 0.99:
            late_mult = 7.3 + 15.0 * ((es - 0.97) / 0.02)
        else:
            late_mult = 22.3 + 30.0 * ((es - 0.99) / 0.01)  # 52× at 100%

        # NO subsampling — use ALL uncovered cells for exact nearest-distance
        ys, xs = np.where(frontier)
        frontier_pts = self.grid_points[ys, xs]

        for i in np.where(self.active_mask > 0.5)[0]:
            d = float(np.min(np.linalg.norm(frontier_pts - self.pos[i][None, :], axis=1)))
            # Penalty starts at scan_range distance — if you can see it, no penalty
            excess = max(0.0, d - self.cfg.scan_range_m)
            # Normalize: penalty reaches 1.0 at ~10m away from nearest uncovered cell
            pen[i] = min(1.0, excess / 10.0) * late_mult
        return pen * self.active_mask

    def _fairness_penalty(self) -> np.ndarray:
        """Penalize drones whose RECENT contribution is significantly below average.

        Uses a proper sliding window (_recent_discoveries) of the last ~20s.
        Historical luck (good spawn position) does NOT shield a drone from
        being penalized for current inactivity.

        Activates after step 50.
        """
        pen = np.zeros((self.max_agents,), dtype=np.float32)
        active_idxs = np.where(self.active_mask > 0.5)[0]
        n = len(active_idxs)
        if n < 2 or self.step_count < 50:
            return pen

        # ── Recent sliding window (PRIMARY signal — 70% weight) ──
        n_filled = min(self._recent_disc_ptr, self._fairness_window)
        if n_filled < 10:
            return pen

        recent_totals = np.sum(self._recent_discoveries[:, :n_filled], axis=1).astype(np.float32)
        recent_active = recent_totals[active_idxs]
        mean_recent = float(np.mean(recent_active))
        recent_pen = np.zeros(n, dtype=np.float32)
        if mean_recent > 0.5:
            for local_idx in range(n):
                deficit = (mean_recent - recent_active[local_idx]) / mean_recent
                if deficit > 0.20:  # 20% below average triggers penalty
                    recent_pen[local_idx] = ((deficit - 0.20) / 0.80) ** 1.0
                    # Extra harsh: if drone discovered ZERO in recent window while others did
                    if recent_active[local_idx] < 1.0 and mean_recent > 3.0:
                        recent_pen[local_idx] = max(recent_pen[local_idx], 0.9)

        # ── Cumulative component (SECONDARY — 30% weight) ──
        discoveries = self._drone_discoveries[active_idxs].astype(np.float32)
        mean_disc = float(np.mean(discoveries))
        max_disc = float(np.max(discoveries))
        cum_pen = np.zeros(n, dtype=np.float32)
        if max_disc >= 5.0:
            for local_idx in range(n):
                deficit = (mean_disc - discoveries[local_idx]) / max(mean_disc, 1.0)
                if deficit > 0.15:
                    cum_pen[local_idx] = ((deficit - 0.15) / 0.85) ** 1.2

        # ── Blend: recent dominates ──
        for local_idx, drone_idx in enumerate(active_idxs):
            pen[drone_idx] = 0.70 * recent_pen[local_idx] + 0.30 * cum_pen[local_idx]

        return pen * self.active_mask

    def _late_game_hunting_bonus(self) -> np.ndarray:
        """Strong per-drone bonus in late game (>93% coverage) for discovering NEW cells.

        This is the key mechanism to reach 100%: when almost everything is covered,
        each drone that scans a previously-unseen cell gets a massive reward burst.
        Also gives bonus to the CLOSEST drone to any uncovered cell cluster to
        encourage territorial assignment.
        """
        bonus = np.zeros((self.max_agents,), dtype=np.float32)
        es = self.last_ever_seen_fraction
        if es < 0.93:
            return bonus

        # Escalation: reward multiplier increases as we approach 100%
        # At 93% → 1x, at 97% → 3x, at 99% → 6x
        escalation = 1.0 + 5.0 * ((es - 0.93) / 0.07) ** 1.5

        # Part 1: Reward drones that discovered cells THIS step
        for i in np.where(self.active_mask > 0.5)[0]:
            if self._step_discoveries[i] > 0:
                bonus[i] += float(self._step_discoveries[i]) * escalation

        # Part 2: Bonus for being the CLOSEST active drone to uncovered cells
        # (encourages the nearest drone to claim and hunt remaining cells)
        frontier = self.active_area_mask & (self.ever_visited == 0)
        if not np.any(frontier):
            return bonus * self.active_mask

        active_idxs = np.where(self.active_mask > 0.5)[0]
        if len(active_idxs) == 0:
            return bonus

        ys, xs = np.where(frontier)
        frontier_pts = self.grid_points[ys, xs]

        # For each uncovered cell, find the closest drone
        drone_positions = self.pos[active_idxs]
        # dists: (n_frontier, n_active)
        dists = np.linalg.norm(
            frontier_pts[:, None, :] - drone_positions[None, :, :], axis=-1
        )
        nearest_drone = np.argmin(dists, axis=1)  # which drone is closest to each cell

        # Count how many uncovered cells each drone is closest to
        for local_idx, drone_idx in enumerate(active_idxs):
            n_assigned = int(np.sum(nearest_drone == local_idx))
            if n_assigned > 0:
                # Bonus: you're the nearest to uncovered cells → go get them!
                # Weighted by proximity (closer = higher bonus)
                my_cells = dists[nearest_drone == local_idx, local_idx]
                proximity = float(np.mean(np.clip(1.0 - my_cells / self.cfg.scan_range_m, 0.0, 1.0)))
                bonus[drone_idx] += proximity * escalation * 0.5

        return bonus * self.active_mask

    def _local_responsibility_bonus(self) -> np.ndarray:
        """Per-drone reward for covering cells where THIS drone is the closest agent.
        Penalizes redundant coverage where another drone is closer.

        Creates a soft, emergent Voronoi-like division of labor:
        - Positive signal for scanning cells in YOUR territory (nearest drone)
        - Negative signal for scanning cells in ANOTHER drone's territory
        - Distance-weighted: closer cells give stronger signal

        Phase-aware: stronger in mid/late game when spreading matters most.
        """
        bonus = np.zeros((self.max_agents,), dtype=np.float32)
        masks = self._scan_masks()
        active_idxs = np.where(self.active_mask > 0.5)[0]
        n = len(active_idxs)
        if n < 2:
            # With 0-1 drones, everything is "yours" — no responsibility signal
            return bonus

        es = self.last_ever_seen_fraction
        # Phase scaling: weak early (exploration matters more), strong mid/late
        if es < 0.50:
            phase_mult = 0.3
        elif es < 0.90:
            phase_mult = 0.3 + 0.7 * ((es - 0.50) / 0.40)  # 0.3 → 1.0
        else:
            phase_mult = 1.0 + 2.0 * ((es - 0.90) / 0.10)  # 1.0 → 3.0

        for i in active_idxs:
            scan_cells = masks[i] & self.active_area_mask
            total = int(np.sum(scan_cells))
            if total < 1:
                continue

            # Get world coordinates of scanned cells
            ys, xs = np.where(scan_cells)
            pts = self.grid_points[ys, xs]  # (K, 2)

            # Distance from each scanned cell to all active drones
            dists_to_all = np.linalg.norm(
                pts[:, None, :] - self.pos[active_idxs][None, :, :], axis=-1
            )  # (K, n_active)

            # Find which drone is closest to each cell
            nearest = np.argmin(dists_to_all, axis=1)  # (K,)
            my_local_idx = int(np.where(active_idxs == i)[0][0])

            # Cells where I am the nearest drone
            mine = nearest == my_local_idx
            n_mine = int(np.sum(mine))

            # Reward: fraction of scan that is in my territory (0-1)
            # This naturally rewards drones for staying in "their" area
            ownership_frac = n_mine / total

            # Distance bonus: weight by proximity (closer owned cells = better)
            if n_mine > 0:
                my_dists = dists_to_all[mine, my_local_idx]
                proximity = float(np.mean(np.clip(
                    1.0 - my_dists / (self.cfg.scan_range_m * 2.0), 0.0, 1.0
                )))
            else:
                proximity = 0.0

            # Penalty component: fraction of scan in another drone's territory
            # (redundant work — another drone is closer and should handle it)
            redundant_frac = 1.0 - ownership_frac

            bonus[i] = (ownership_frac * proximity - 0.5 * redundant_frac) * phase_mult

        return bonus * self.active_mask

    def _hole_left_behind_penalty(self) -> np.ndarray:
        """Penalize drones that are moving AWAY from nearby uncovered cells.

        Key mechanism for "no hole left behind":
        - If uncovered cells exist within extended scan range (2× scan_range),
          the drone MUST NOT increase distance to them.
        - Penalty scales with:
          (a) How many uncovered cells are nearby
          (b) Whether the drone moved AWAY from them (positive distance change)
          (c) Phase: massive in late game (>90%), moderate earlier

        This creates an irresistible "gravity well" around uncovered cells
        that prevents drones from leaving holes behind.
        """
        pen = np.zeros((self.max_agents,), dtype=np.float32)
        es = self.last_ever_seen_fraction

        # Phase-aware scaling
        if es < 0.50:
            phase_mult = 0.1  # very mild early — exploration takes priority
        elif es < 0.90:
            phase_mult = 0.1 + 0.9 * ((es - 0.50) / 0.40)  # 0.1 → 1.0
        elif es < 0.97:
            phase_mult = 1.0 + 4.0 * ((es - 0.90) / 0.07)  # 1.0 → 5.0
        else:
            phase_mult = 5.0 + 10.0 * ((es - 0.97) / 0.03)  # 5.0 → 15.0

        search_r_cells = int(self.cfg.scan_range_m * 2.5 / self.cfg.grid_resolution_m) + 1

        for i in np.where(self.active_mask > 0.5)[0]:
            # Find uncovered cells near current position
            ix, iy = self._world_to_grid(self.pos[i:i + 1])
            ix, iy = int(ix[0]), int(iy[0])
            y_lo = max(0, iy - search_r_cells)
            y_hi = min(self.grid_h, iy + search_r_cells + 1)
            x_lo = max(0, ix - search_r_cells)
            x_hi = min(self.grid_w, ix + search_r_cells + 1)

            active_patch = self.active_area_mask[y_lo:y_hi, x_lo:x_hi]
            # Use _ever_visited_before (before this step's coverage update)
            ever_patch = self._ever_visited_before[y_lo:y_hi, x_lo:x_hi]
            uncov = (ever_patch == 0) & active_patch
            n_uncov = int(np.sum(uncov))

            if n_uncov == 0:
                continue

            # Compute centroid of nearby uncovered cells
            uncov_ys, uncov_xs = np.where(uncov)
            world_xs = (uncov_xs + x_lo + 0.5) * self.cfg.grid_resolution_m - self.cfg.world_w / 2.0
            world_ys = (uncov_ys + y_lo + 0.5) * self.cfg.grid_resolution_m - self.cfg.world_h / 2.0
            centroid = np.array([float(np.mean(world_xs)), float(np.mean(world_ys))], dtype=np.float32)

            # Did drone move AWAY from the uncovered centroid?
            prev_dist = float(np.linalg.norm(self.prev_pos[i] - centroid))
            curr_dist = float(np.linalg.norm(self.pos[i] - centroid))
            retreat = max(0.0, curr_dist - prev_dist)  # positive = moving away

            if retreat < 1e-4:
                continue  # not retreating — no penalty

            # Scale by how many uncovered cells (more holes = worse)
            total_local = max(1, int(np.sum(active_patch)))
            hole_severity = min(1.0, n_uncov / (total_local * 0.3))

            # Scale by retreat speed (fast retreat = worse)
            retreat_norm = min(1.0, retreat / (self.cfg.max_speed * self.dt))

            pen[i] = hole_severity * retreat_norm * phase_mult

        return pen * self.active_mask

    def _maintenance_need_reduction_bonus(self) -> np.ndarray:
        """Per-drone reward for MEANINGFUL maintenance: need reduction, not activity.

        Returns per-drone bonus proportional to how much maintenance NEED the drone
        reduced this step. A drone scanning a high-need cell (stale, near corner,
        not recently rescanned) gets a large bonus. A drone rescanning a cell it
        just visited gets nearly zero.

        COMPLETION-GATED: only fires after 100% coverage is reached.
        Before that, exploration is the priority and this returns zeros.

        The diminishing returns from _maint_recent_scans ensure:
        - Circling the same area rapidly → accumulator grows → gain → 0
        - Visiting diverse stale areas → accumulator low → gain stays high
        No hard cutoff, continuous exponential decay.
        """
        bonus = np.zeros((self.max_agents,), dtype=np.float32)

        # Completion gate: only meaningful after 100% coverage
        if not self._milestone_hit.get(100, False):
            return bonus

        # Use pre-computed per-drone maintenance gain from _update_coverage
        total_gain = float(np.sum(self._step_maint_gain))
        if total_gain < 1e-6:
            return bonus

        # Normalize: each drone's gain relative to total possible need
        total_need = float(np.sum(self._maint_need_before))
        if total_need < 1e-6:
            return bonus

        for i in np.where(self.active_mask > 0.5)[0]:
            # Normalized gain: fraction of total need this drone reduced
            bonus[i] = self._step_maint_gain[i] / max(1.0, total_need)

        return bonus * self.active_mask

    def get_maintenance_quality_score(self) -> dict:
        """Compute maintenance quality metrics for evaluation.

        Returns a dict with:
        - 'mean_need': average maintenance need across all active cells (lower = better maintained)
        - 'max_need': worst-case cell need (identifies neglected areas)
        - 'need_std': std of need across cells (lower = more uniform maintenance)
        - 'need_gini': Gini coefficient of per-drone maintenance gains (fairness)
        - 'total_gain': total maintenance gain accumulated this episode
        - 'quality_score': composite maintenance quality [0, 1]

        This is used ONLY for evaluation, never for reward.
        """
        active_need = self._maint_need_map[self.active_area_mask & (self.ever_visited > 0)]

        if active_need.size == 0:
            return {
                'mean_need': 0.0, 'max_need': 0.0, 'need_std': 0.0,
                'need_gini': 0.0, 'total_gain': 0.0, 'quality_score': 1.0,
            }

        mean_need = float(np.mean(active_need))
        max_need = float(np.max(active_need))
        need_std = float(np.std(active_need))

        # Fairness of maintenance work
        active_idxs = np.where(self.active_mask > 0.5)[0]
        gains = self._drone_maint_gains[active_idxs].astype(np.float64)
        n = len(gains)
        if n > 1 and np.sum(gains) > 0:
            sorted_g = np.sort(gains)
            index = np.arange(1, n + 1, dtype=np.float64)
            gini = float((2.0 * np.sum(index * sorted_g) - (n + 1) * np.sum(sorted_g)) /
                         (n * np.sum(sorted_g)))
        else:
            gini = 0.0

        total_gain = float(np.sum(self._drone_maint_gains))

        # Composite quality: low mean need + low max need + low std + fair distribution
        # All components in [0, 1] where 1 = best
        # mean_need typically in [0, ~2]; normalize by geometry-boosted max
        max_possible_need = float(np.max(self._maint_geometry_weight)) if np.any(self._maint_geometry_weight > 0) else 1.0
        mean_score = max(0.0, 1.0 - mean_need / max(0.5, max_possible_need))
        max_score = max(0.0, 1.0 - max_need / max(1.0, max_possible_need * 2.0))
        uniformity = max(0.0, 1.0 - need_std / max(0.3, mean_need + 0.01))
        fairness = max(0.0, 1.0 - gini * 2.0)

        quality = 0.30 * mean_score + 0.25 * max_score + 0.20 * uniformity + 0.25 * fairness

        return {
            'mean_need': mean_need,
            'max_need': max_need,
            'need_std': need_std,
            'need_gini': gini,
            'total_gain': total_gain,
            'quality_score': float(quality),
        }

    def _corner_scan_bonus(self) -> np.ndarray:
        """Reward drones that are near polygon corners AND scanning uncovered cells there.

        Only fires when ever_seen_fraction > 0.90 (late-game corner cleanup).
        Returns per-agent bonus in [0, 1].
        """
        bonus = np.zeros((self.max_agents,), dtype=np.float32)
        es = self.last_ever_seen_fraction
        if es < 0.75:
            return bonus  # activate earlier for corner-cleanup phase (was 0.90)

        # Scale up as we approach 100% — the last few % are critical
        urgency = min(1.0, (es - 0.75) / 0.15)  # 0 at 75%, 1 at 90%+

        # Find uncovered cells near polygon vertices
        uncovered = self.active_area_mask & (self.ever_visited == 0)
        if not np.any(uncovered):
            return bonus

        uncov_pts = self.grid_points[uncovered]
        # Distance from each uncovered cell to nearest polygon vertex
        vert_dists = np.min(np.linalg.norm(
            uncov_pts[:, None, :] - self.polygon_vertices[None, :, :], axis=-1
        ), axis=1)
        # Cells within corner_scan_distance of a vertex are "corner cells"
        corner_mask = vert_dists < self.cfg.corner_scan_distance
        if not np.any(corner_mask):
            # No corner cells uncovered — reward being near ANY uncovered cell
            corner_pts = uncov_pts
        else:
            corner_pts = uncov_pts[corner_mask]

        for i in np.where(self.active_mask > 0.5)[0]:
            d = np.min(np.linalg.norm(corner_pts - self.pos[i][None, :], axis=1))
            # Reward inversely proportional to distance (peaks when on top of corner cells)
            bonus[i] = max(0.0, 1.0 - d / self.cfg.corner_scan_distance) * urgency
        return bonus * self.active_mask

    def _frontier_progress(self) -> np.ndarray:
        out = np.zeros((self.max_agents,), dtype=np.float32)
        frontier = self.active_area_mask & (self.ever_visited == 0)
        if not np.any(frontier):
            return out
        active_idxs = np.where(self.active_mask > 0.5)[0]
        if len(active_idxs) == 0:
            return out
        ys, xs = np.where(frontier)
        step = max(1, len(xs) // 300)
        pts = self.grid_points[ys[::step], xs[::step]]  # (P, 2)
        # Batch: (n_active, P) distances
        prev_dists = np.linalg.norm(pts[None, :, :] - self.prev_pos[active_idxs, None, :], axis=-1)
        now_dists = np.linalg.norm(pts[None, :, :] - self.pos[active_idxs, None, :], axis=-1)
        out[active_idxs] = np.maximum(0.0, prev_dists.min(axis=1) - now_dists.min(axis=1))
        return out

    def _world_to_grid(self, pts: np.ndarray):
        x = (pts[:, 0] + self.cfg.world_w / 2.0) / self.cfg.grid_resolution_m - 0.5
        y = (pts[:, 1] + self.cfg.world_h / 2.0) / self.cfg.grid_resolution_m - 0.5
        ix = np.clip(np.round(x).astype(int), 0, self.grid_w - 1)
        iy = np.clip(np.round(y).astype(int), 0, self.grid_h - 1)
        return ix, iy

    def _uncovered_near_agent(self, agent_idx: int):
        """Count uncovered active cells within 2× scan_range of an agent.
        Returns (count, unit_direction_to_nearest, distance_to_nearest).
        Operates at FULL RESOLUTION — no subsampling."""
        margin_cells = int(self.cfg.scan_range_m * 2.5 / self.cfg.grid_resolution_m) + 1
        ix, iy = self._world_to_grid(self.pos[agent_idx:agent_idx + 1])
        ix, iy = int(ix[0]), int(iy[0])
        y_lo = max(0, iy - margin_cells)
        y_hi = min(self.grid_h, iy + margin_cells + 1)
        x_lo = max(0, ix - margin_cells)
        x_hi = min(self.grid_w, ix + margin_cells + 1)
        active_patch = self.active_area_mask[y_lo:y_hi, x_lo:x_hi]
        ever_patch = self.ever_visited[y_lo:y_hi, x_lo:x_hi]
        uncov = (ever_patch == 0) & active_patch
        count = int(np.sum(uncov))
        if count == 0:
            return 0, np.zeros(2, dtype=np.float32), 999.0
        # Find nearest uncovered cell
        uncov_ys, uncov_xs = np.where(uncov)
        # Convert to world coordinates
        world_xs = (uncov_xs + x_lo + 0.5) * self.cfg.grid_resolution_m - self.cfg.world_w / 2.0
        world_ys = (uncov_ys + y_lo + 0.5) * self.cfg.grid_resolution_m - self.cfg.world_h / 2.0
        pts = np.stack([world_xs, world_ys], axis=-1)
        dists = np.linalg.norm(pts - self.pos[agent_idx][None, :], axis=1)
        nearest_idx = np.argmin(dists)
        nearest_dist = float(dists[nearest_idx])
        delta = pts[nearest_idx] - self.pos[agent_idx]
        d = np.linalg.norm(delta)
        direction = delta / d if d > 1e-3 else np.zeros(2, dtype=np.float32)
        return count, direction, nearest_dist

    def _crop_channel(self, channel: np.ndarray, ix: int, iy: int) -> np.ndarray:
        r = self.local_radius
        padded = np.pad(channel, ((r, r), (r, r)), mode='constant')
        return padded[iy:iy + self.local_map_size, ix:ix + self.local_map_size].astype(np.float32)

    def _egocentric_crop(self, channel: np.ndarray, agent_idx: int) -> np.ndarray:
        """Crop a local_map_size × local_map_size patch centered on the agent,
        rotated so the agent's heading points UP (pure egocentric).

        - Agent is always at the center pixel
        - Forward direction = up in the output image
        - No absolute/global spatial reference
        - Out-of-bounds pixels are 0
        """
        s = self.local_map_size
        r = self.local_radius  # s // 2

        # Agent position in continuous grid coordinates
        px = (self.pos[agent_idx, 0] + self.cfg.world_w / 2.0) / self.cfg.grid_resolution_m - 0.5
        py = (self.pos[agent_idx, 1] + self.cfg.world_h / 2.0) / self.cfg.grid_resolution_m - 0.5

        heading = float(self.heading[agent_idx])
        cos_h = math.cos(heading)
        sin_h = math.sin(heading)

        # Output pixel grid: (oy, ox) in [0, s)
        # ego_right  = ox - r   (positive = agent's right)
        # ego_forward = r - oy  (positive = agent's forward / up in image)
        oy, ox = np.mgrid[0:s, 0:s]
        ego_right = (ox - r).astype(np.float32)
        ego_forward = (r - oy).astype(np.float32)

        # Transform to world grid coordinates:
        #   forward in world = (cos_h, sin_h)
        #   right   in world = (sin_h, -cos_h)
        src_x = px + cos_h * ego_forward + sin_h * ego_right
        src_y = py + sin_h * ego_forward - cos_h * ego_right

        # Nearest-neighbour sampling (out-of-bounds → 0)
        src_ix = np.round(src_x).astype(int)
        src_iy = np.round(src_y).astype(int)
        valid = (src_ix >= 0) & (src_ix < self.grid_w) & (src_iy >= 0) & (src_iy < self.grid_h)
        src_ix_safe = np.clip(src_ix, 0, self.grid_w - 1)
        src_iy_safe = np.clip(src_iy, 0, self.grid_h - 1)

        result = channel[src_iy_safe, src_ix_safe].astype(np.float32)
        result[~valid] = 0.0
        return result

    def _forward_sensor_channel(self, agent_idx: int) -> np.ndarray:
        half_fov = math.radians(self.cfg.forward_sensor_fov_deg) / 2.0
        d = (self._fwd_ang - float(self.heading[agent_idx]) + math.pi) % (2.0 * math.pi) - math.pi
        return (self._fwd_range_mask & (np.abs(d) <= half_fov)).astype(np.float32)

    def _compute_frontier_waypoints(self) -> np.ndarray:
        """Compute a target waypoint per drone toward unexplored/stale areas.

        Uses Voronoi partitioning: each unexplored cell is assigned to its
        nearest drone, then each drone's waypoint is the centroid of its
        assigned unexplored cells.  This naturally spreads drones apart and
        directs each one toward *its own* frontier region.

        Returns (max_agents, 3): [dir_x, dir_y, normalized_distance]
           dir_x, dir_y = unit vector from drone toward its waypoint
           normalized_distance = distance / world_diagonal (0=on target, 1=far)
        """
        result = np.zeros((self.max_agents, 3), dtype=np.float32)
        active_idxs = np.where(self.active_mask > 0.5)[0]
        if len(active_idxs) == 0:
            return result

        # Priority 1: unexplored cells.  Priority 2: stale cells.
        frontier = self.active_area_mask & (self.ever_visited == 0)
        if not np.any(frontier):
            frontier = self.active_area_mask & (self.coverage_value < self.cfg.maintained_threshold)
        if not np.any(frontier):
            # Everything is well-covered — no waypoint needed
            return result

        ys, xs = np.where(frontier)
        # Subsample for speed (max ~500 points)
        step = max(1, len(xs) // 500)
        xs, ys = xs[::step], ys[::step]
        frontier_pts = self.grid_points[ys, xs]  # (N, 2)

        # Late-game: weight frontier cells near corners higher
        es = self.last_ever_seen_fraction
        if es > 0.90 and len(frontier_pts) > 0:
            vert_dists = np.min(np.linalg.norm(
                frontier_pts[:, None, :] - self.polygon_vertices[None, :, :], axis=-1
            ), axis=1)
            corner_weight = np.clip(1.0 - vert_dists / self.cfg.corner_scan_distance, 0.0, 1.0)
            # Duplicate corner-adjacent frontier points to increase their pull
            corner_mask = corner_weight > 0.3
            if np.any(corner_mask):
                corner_pts = frontier_pts[corner_mask]
                # Triple the corner points to make waypoints gravitate there
                frontier_pts = np.vstack([frontier_pts, corner_pts, corner_pts])

        # Assign each frontier cell to nearest active drone (Voronoi)
        drone_positions = self.pos[active_idxs]  # (n_active, 2)
        # dists: (N_frontier, n_active)
        dists = np.linalg.norm(
            frontier_pts[:, None, :] - drone_positions[None, :, :], axis=-1
        )
        assignments = np.argmin(dists, axis=1)  # which drone owns each cell

        max_world = math.sqrt(self.cfg.world_w ** 2 + self.cfg.world_h ** 2)

        for local_idx, drone_idx in enumerate(active_idxs):
            owned_mask = assignments == local_idx
            if not np.any(owned_mask):
                # No cells assigned — fall back to nearest frontier cell
                all_dists = dists[:, local_idx]
                nearest = np.argmin(all_dists)
                waypoint = frontier_pts[nearest]
            else:
                # Waypoint = centroid of owned frontier cells
                owned_pts = frontier_pts[owned_mask]
                # Weight by inverse distance — prefer closer unexplored cells
                d = np.linalg.norm(owned_pts - self.pos[drone_idx], axis=1)
                weights = 1.0 / (d + 0.5)  # soft inverse, avoid div-by-zero
                waypoint = np.average(owned_pts, axis=0, weights=weights)

            delta = waypoint - self.pos[drone_idx]
            dist = float(np.linalg.norm(delta))
            if dist > 1e-3:
                result[drone_idx, 0] = delta[0] / dist  # unit dir x
                result[drone_idx, 1] = delta[1] / dist  # unit dir y
            result[drone_idx, 2] = min(dist / max_world, 1.0)  # normalized dist

        return result

    def _frontier_attraction_map(self) -> np.ndarray:
        """Create a blurred map where unseen/stale cells glow — gives the CNN
        a gradient toward unexplored areas.  Cached per step."""
        if self._cached_frontier_map_step == self.step_count and self._cached_frontier_map is not None:
            return self._cached_frontier_map
        frontier = ((1.0 - self.ever_visited.astype(np.float32)) * self.active_area_mask.astype(np.float32))
        stale = ((self.coverage_value < self.cfg.maintained_threshold) & self.active_area_mask & (self.ever_visited > 0)).astype(np.float32) * 0.5
        raw = frontier + stale
        # Fast box blur using cumsum (2 passes of size 5)
        blurred = self._fast_box_blur(raw, 5)
        blurred = self._fast_box_blur(blurred, 5)
        mx = blurred.max()
        if mx > 1e-6:
            blurred /= mx
        self._cached_frontier_map = blurred.astype(np.float32)
        self._cached_frontier_map_step = self.step_count
        return self._cached_frontier_map

    @staticmethod
    def _fast_box_blur(arr: np.ndarray, k: int) -> np.ndarray:
        """Fast 2D box blur using cumulative sums. No scipy needed."""
        h, w = arr.shape
        r = k // 2
        padded = np.pad(arr, r, mode='constant')
        # Horizontal pass
        cs = np.cumsum(padded, axis=1)
        h_blur = (cs[:, k:] - cs[:, :-k]) / k
        # Vertical pass
        cs2 = np.cumsum(h_blur, axis=0)
        return (cs2[k:, :] - cs2[:-k, :]) / k

    def _downsample_to_mapsize(self, full_map: np.ndarray) -> np.ndarray:
        """Downsample a (grid_h, grid_w) map to (local_map_size, local_map_size).
        Fast vectorized area-average using reduceat."""
        s = self.local_map_size
        h, w = full_map.shape
        # Row and column bin edges
        row_edges = np.linspace(0, h, s + 1).astype(int)
        col_edges = np.linspace(0, w, s + 1).astype(int)
        # reduceat along rows
        row_sums = np.add.reduceat(full_map, row_edges[:-1], axis=0)  # (s, w)
        # reduceat along cols
        block_sums = np.add.reduceat(row_sums, col_edges[:-1], axis=1)  # (s, s)
        # Compute block sizes for proper averaging
        row_sizes = np.diff(row_edges)  # (s,)
        col_sizes = np.diff(col_edges)  # (s,)
        block_counts = row_sizes[:, None] * col_sizes[None, :]  # (s, s)
        return (block_sums / np.maximum(block_counts, 1)).astype(np.float32)

    def _downsample_max_to_mapsize(self, full_map: np.ndarray) -> np.ndarray:
        """Downsample a (grid_h, grid_w) map to (local_map_size, local_map_size).
        Fast vectorized max-pooling using reduceat."""
        s = self.local_map_size
        h, w = full_map.shape
        row_edges = np.linspace(0, h, s + 1).astype(int)
        col_edges = np.linspace(0, w, s + 1).astype(int)
        # reduceat along rows
        row_maxs = np.maximum.reduceat(full_map, row_edges[:-1], axis=0)  # (s, w)
        # reduceat along cols
        block_maxs = np.maximum.reduceat(row_maxs, col_edges[:-1], axis=1)  # (s, s)
        return block_maxs.astype(np.float32)

    def _forward_sensor_channel_global(self, agent_idx: int) -> np.ndarray:
        """FOV cone rendered on the GLOBAL downsampled map.
        Uses an EXTENDED range (3× scan_range) so the cone is clearly visible
        after downsampling. Intensity fades with distance (1.0 at drone, 0.0 at edge)."""
        s = self.local_map_size
        if self.active_mask[agent_idx] <= 0.5:
            return np.zeros((s, s), dtype=np.float32)
        # Use extended range for visibility (3× scan range = ~7.5m on default config)
        vis_range = self.cfg.scan_range_m * 3.0
        half_fov = math.radians(self.cfg.scan_fov_deg) / 2.0
        pos = self.pos[agent_idx]
        heading = float(self.heading[agent_idx])
        # Vector from drone to each grid cell
        rel = self.grid_points - pos[None, None, :]  # (H, W, 2)
        dist = np.sqrt(rel[..., 0] ** 2 + rel[..., 1] ** 2)
        angle = np.arctan2(rel[..., 1], rel[..., 0]) - heading
        angle = (angle + np.pi) % (2.0 * np.pi) - np.pi
        in_cone = (dist <= vis_range) & (np.abs(angle) <= half_fov) & (dist > 0.01)
        # Distance gradient: 1.0 at drone position, fading to 0.2 at edge of cone
        fov_map = np.where(in_cone, 1.0 - 0.8 * (dist / vis_range), 0.0).astype(np.float32)
        return self._downsample_to_mapsize(fov_map)

    def _forward_sensor_channel_full(self, agent_idx: int) -> np.ndarray:
        """FOV cone rendered at FULL RESOLUTION on the world grid.
        No downsampling — preserves every cell. Used for egocentric actor maps."""
        if self.active_mask[agent_idx] <= 0.5:
            return np.zeros((self.grid_h, self.grid_w), dtype=np.float32)
        vis_range = self.cfg.scan_range_m * 2.0  # moderate range for local view
        half_fov = math.radians(self.cfg.scan_fov_deg) / 2.0
        pos = self.pos[agent_idx]
        heading = float(self.heading[agent_idx])
        # Only compute in a local patch around the agent for efficiency
        margin_cells = int(vis_range / self.cfg.grid_resolution_m) + 2
        ix, iy = self._world_to_grid(self.pos[agent_idx:agent_idx + 1])
        cx, cy = int(ix[0]), int(iy[0])
        y_lo = max(0, cy - margin_cells)
        y_hi = min(self.grid_h, cy + margin_cells + 1)
        x_lo = max(0, cx - margin_cells)
        x_hi = min(self.grid_w, cx + margin_cells + 1)
        result = np.zeros((self.grid_h, self.grid_w), dtype=np.float32)
        patch_pts = self.grid_points[y_lo:y_hi, x_lo:x_hi]  # (ph, pw, 2)
        rel = patch_pts - pos[None, None, :]
        dist = np.sqrt(rel[..., 0] ** 2 + rel[..., 1] ** 2)
        angle = np.arctan2(rel[..., 1], rel[..., 0]) - heading
        angle = (angle + np.pi) % (2.0 * np.pi) - np.pi
        in_cone = (dist <= vis_range) & (np.abs(angle) <= half_fov) & (dist > 0.01)
        fov_vals = np.where(in_cone, 1.0 - 0.8 * (dist / vis_range), 0.0)
        result[y_lo:y_hi, x_lo:x_hi] = fov_vals.astype(np.float32)
        return result

    def _draw_disk(self, grid: np.ndarray, cx: int, cy: int, radius_cells: int, value: float = 1.0):
        """Draw a filled disk on the grid at (cx, cy) with given radius in cells."""
        y_lo = max(0, cy - radius_cells)
        y_hi = min(self.grid_h, cy + radius_cells + 1)
        x_lo = max(0, cx - radius_cells)
        x_hi = min(self.grid_w, cx + radius_cells + 1)
        for y in range(y_lo, y_hi):
            for x in range(x_lo, x_hi):
                if (x - cx) ** 2 + (y - cy) ** 2 <= radius_cells ** 2:
                    grid[y, x] = max(grid[y, x], value)

    def get_actor_observation(self):
        ix, iy = self._world_to_grid(self.pos)
        s = self.local_map_size
        maps = np.zeros((self.max_agents, self.map_channels, s, s), dtype=np.float32)
        self_state = np.zeros((self.max_agents, self.self_state_dim), dtype=np.float32)
        neighbor_state = np.zeros((self.max_agents, self.max_neighbors, self.neighbor_state_dim), dtype=np.float32)
        neighbor_mask = np.zeros((self.max_agents, self.max_neighbors), dtype=np.float32)

        # Drone marker radius in grid cells — at full resolution a radius-1 dot is visible
        drone_marker_radius = 1

        # ── Build FULL-RESOLUTION source maps (shared) ──
        # Terrain & Coverage map: -1.0 for walls, 0.0 for never visited, 0.1 to 1.0 for visited
        terrain_map = np.full((self.grid_h, self.grid_w), -1.0, dtype=np.float32)
        terrain_map[self.active_area_mask] = 0.0
        visited_mask = self.active_area_mask & (self.ever_visited > 0)
        terrain_map[visited_mask] = 0.1 + 0.9 * self.coverage_value[visited_mask]

        # For self state stats (keeping these pure for metric calculations)
        never_map = (1.0 - self.ever_visited.astype(np.float32)) * self.active_area_mask.astype(np.float32)
        active_map = self.active_area_mask.astype(np.float32)

        # Trajectory heatmap — ALL drones combined, FULL RESOLUTION
        traj_raw = self._trajectory_heatmap
        traj_map = np.maximum(
            np.maximum(traj_raw, np.roll(traj_raw, 1, axis=0)),
            np.maximum(np.roll(traj_raw, -1, axis=0),
                       np.maximum(np.roll(traj_raw, 1, axis=1), np.roll(traj_raw, -1, axis=1)))
        )

        for i in range(self.max_agents):
            # ─── ALL channels are EGOCENTRIC: local crop + rotated by heading ───
            # Ch0: Terrain & Coverage (-1 walls, 0 never, 0.1-1.0 freshness)
            maps[i, 0] = self._egocentric_crop(terrain_map, i)

            # ── Self state vector ──
            edge = distance_to_polygon_edge(self.pos[i], self.polygon_vertices) if self.active_mask[i] > 0.5 else 0.0
            local_cov = self._egocentric_crop(self.coverage_value, i)
            local_never = self._egocentric_crop(never_map, i)
            local_active = self._egocentric_crop(active_map, i)
            corner_dist = float(np.min(np.linalg.norm(self.polygon_vertices - self.pos[i], axis=1))) if self.active_mask[i] > 0.5 else 0.0
            corner_proximity = max(0.0, 1.0 - corner_dist / self.cfg.corner_scan_distance)
            
            # Explicitly compute traj density for the self_state without adding it to CNN channels
            ego_traj = self._egocentric_crop(traj_map, i)
            traj_density = float(np.mean(ego_traj))

            # ── Full-resolution nearest-uncovered and local-uncovered signals ──
            if self.active_mask[i] > 0.5:
                _uncov_count, _uncov_dir, _uncov_dist = self._uncovered_near_agent(i)
                nearest_uncov_norm = min(1.0, _uncov_dist / max(1e-6, self.cfg.scan_range_m * 5.0))  # 0=on top, 1=far
                # Local uncovered fraction in scan range (full res, no downsample)
                scan_r_cells = int(self.cfg.scan_range_m / self.cfg.grid_resolution_m)
                _ix, _iy = self._world_to_grid(self.pos[i:i + 1])
                _ix, _iy = int(_ix[0]), int(_iy[0])
                _ylo = max(0, _iy - scan_r_cells)
                _yhi = min(self.grid_h, _iy + scan_r_cells + 1)
                _xlo = max(0, _ix - scan_r_cells)
                _xhi = min(self.grid_w, _ix + scan_r_cells + 1)
                _ap = self.active_area_mask[_ylo:_yhi, _xlo:_xhi]
                _total_a = max(1, int(np.sum(_ap)))
                _uncov_local = float(np.sum((self.ever_visited[_ylo:_yhi, _xlo:_xhi] == 0) & _ap)) / _total_a
                # Recent contribution rate (my share of recent discoveries)
                n_filled = min(self._recent_disc_ptr, self._fairness_window)
                my_recent = float(np.sum(self._recent_discoveries[i, :max(1, n_filled)]))
                total_recent = float(np.sum(self._recent_discoveries[:, :max(1, n_filled)]))
                recent_contrib_rate = my_recent / max(1.0, total_recent)
            else:
                nearest_uncov_norm = 1.0
                _uncov_local = 0.0
                recent_contrib_rate = 0.0

            # Ego-frame velocity: project onto forward/right axes
            cos_h = math.cos(float(self.heading[i]))
            sin_h = math.sin(float(self.heading[i]))
            vel_forward = self.vel[i, 0] * cos_h + self.vel[i, 1] * sin_h
            vel_right = self.vel[i, 0] * sin_h - self.vel[i, 1] * cos_h

            self_state[i] = np.array([
                self.pos[i, 0] / (self.cfg.world_w / 2.0),        # 0: global x (scalar — allowed)
                self.pos[i, 1] / (self.cfg.world_h / 2.0),        # 1: global y (scalar — allowed)
                vel_forward / max(1e-6, self.cfg.max_speed),       # 2: ego forward speed
                vel_right / max(1e-6, self.cfg.max_speed),         # 3: ego lateral speed
                math.sin(float(self.heading[i])),                  # 4
                math.cos(float(self.heading[i])),                  # 5
                edge / max(1e-6, max(self.cfg.world_w, self.cfg.world_h)),  # 6
                self.step_count / max(1, self.max_steps),          # 7
                float(np.mean(local_cov)),                         # 8: local ego coverage mean
                float(np.min(local_cov)),                          # 9: local ego coverage min
                float(np.max(local_cov)),                          # 10: local ego coverage max
                float(np.mean(local_never)),                       # 11: local ego frontier density
                float(np.mean(local_active)),                      # 12: local ego active area frac
                float(np.sum(self.active_mask) / self.max_agents), # 13
                self.active_mask[i],                               # 14
                float(np.mean(np.abs(self.prev_actions[i]))),      # 15
                corner_proximity,    # 16: 1.0 = at corner, 0.0 = far from corners
                traj_density,        # 17: how visited local area is (avoid revisit)
                nearest_uncov_norm,  # 18: distance to nearest uncovered cell (0=on it, 1=far) — FULL RES
                _uncov_local,        # 19: fraction of local cells uncovered (full res, no downsample)
                recent_contrib_rate, # 20: my share of recent team discoveries (fairness signal)
            ], dtype=np.float32)

            # ── Neighbor state (relative positions in EGO frame) ──
            neighbors = []
            for j in range(self.max_agents):
                if i == j:
                    continue
                rel = self.pos[j] - self.pos[i]
                dist = float(np.linalg.norm(rel))

                # Rotate relative position into ego frame (forward, right)
                rel_ego_fwd = rel[0] * cos_h + rel[1] * sin_h
                rel_ego_rgt = rel[0] * sin_h - rel[1] * cos_h

                # Rotate neighbor velocity into ego frame
                nvel_fwd = self.vel[j, 0] * cos_h + self.vel[j, 1] * sin_h
                nvel_rgt = self.vel[j, 0] * sin_h - self.vel[j, 1] * cos_h

                # Relative heading (neighbor heading - my heading)
                rel_heading = float(self.heading[j]) - float(self.heading[i])
                rel_heading = (rel_heading + math.pi) % (2.0 * math.pi) - math.pi

                # Communicated trajectory direction (where drone j has been heading)
                n_filled = min(self._pos_hist_ptr, self._displacement_window)
                if n_filled >= 2 and self.active_mask[j] > 0.5:
                    oldest = (self._pos_hist_ptr - n_filled) % self._displacement_window
                    traj_delta = self.pos[j] - self._pos_history[j, oldest]
                    traj_len = float(np.linalg.norm(traj_delta))
                    if traj_len > 1e-3:
                        traj_dir = traj_delta / traj_len
                    else:
                        traj_dir = np.zeros(2, dtype=np.float32)
                else:
                    traj_dir = np.zeros(2, dtype=np.float32)

                # Rotate traj_dir into ego frame
                traj_ego_fwd = traj_dir[0] * cos_h + traj_dir[1] * sin_h
                traj_ego_rgt = traj_dir[0] * sin_h - traj_dir[1] * cos_h

                neighbors.append((dist, np.array([
                    rel_ego_fwd / max(1e-6, self.cfg.world_w),     # ego-frame forward offset
                    rel_ego_rgt / max(1e-6, self.cfg.world_h),     # ego-frame right offset
                    nvel_fwd / max(1e-6, self.cfg.max_speed),      # ego-frame neighbor forward vel
                    nvel_rgt / max(1e-6, self.cfg.max_speed),      # ego-frame neighbor right vel
                    dist / max(1e-6, max(self.cfg.world_w, self.cfg.world_h)),
                    self.active_mask[j],
                    math.sin(rel_heading),  # relative heading sin
                    math.cos(rel_heading),  # relative heading cos
                    traj_ego_fwd,           # ego-frame trajectory dir forward
                    traj_ego_rgt,           # ego-frame trajectory dir right
                ], dtype=np.float32)))
            neighbors.sort(key=lambda x: x[0])
            for k, (_d, feat) in enumerate(neighbors[:self.max_neighbors]):
                neighbor_state[i, k] = feat
                neighbor_mask[i, k] = feat[5]  # active_mask field
        return {
            'maps': maps,
            'self_state': self_state,
            'neighbor_state': neighbor_state,
            'neighbor_mask': neighbor_mask,
            'active_mask': self.active_mask.astype(np.float32).copy(),
        }

    def _downsample(self, arr: np.ndarray, out_h: int, out_w: int) -> np.ndarray:
        h, w = arr.shape
        bh = h // out_h
        bw = w // out_w
        trimmed = arr[:out_h * bh, :out_w * bw]
        return trimmed.reshape(out_h, bh, out_w, bw).mean(axis=(1, 3)).astype(np.float32)

    def _downsample_max(self, arr: np.ndarray, out_h: int, out_w: int) -> np.ndarray:
        """Max-pooling downsample: preserves ANY nonzero cell in each block.
        Critical for sparse binary maps (uncovered, stale, drone positions)
        where averaging would hide single cells."""
        h, w = arr.shape
        bh = h // out_h
        bw = w // out_w
        trimmed = arr[:out_h * bh, :out_w * bw]
        return trimmed.reshape(out_h, bh, out_w, bw).max(axis=(1, 3)).astype(np.float32)

    def get_critic_observation(self) -> dict[str, np.ndarray]:
        ds = self.cfg.critic_downsample
        active_area = self.active_area_mask.astype(np.float32)
        uncovered = (1.0 - self.ever_visited.astype(np.float32)) * active_area
        stale = ((self.coverage_value < self.cfg.maintained_threshold) & self.active_area_mask).astype(np.float32)
        drone_map = np.zeros((self.grid_h, self.grid_w), dtype=np.float32)
        ix, iy = self._world_to_grid(self.pos)
        for i in np.where(self.active_mask > 0.5)[0]:
            self._draw_disk(drone_map, ix[i], iy[i], 1, 1.0)

        # Use MAX-pooling for binary/sparse channels (uncovered, stale, drones)
        # so that a single uncovered cell is NEVER averaged away to zero
        global_maps = np.stack([
            self._downsample(self.coverage_value, ds, ds),           # Ch0: mean coverage (ok to average)
            self._downsample_max(uncovered, ds, ds),                  # Ch1: uncovered — MAX preserves any 1
            self._downsample_max(stale, ds, ds),                      # Ch2: stale — MAX preserves any 1
            self._downsample(active_area, ds, ds),                   # Ch3: active area (ok to average)
            self._downsample_max(drone_map, ds, ds),                  # Ch4: drone positions — MAX preserves dot
        ], axis=0).astype(np.float32)

        agent_feats = []
        for i in range(self.max_agents):
            edge = distance_to_polygon_edge(self.pos[i], self.polygon_vertices) if self.active_mask[i] > 0.5 else 0.0
            agent_feats.extend([
                self.pos[i, 0] / (self.cfg.world_w / 2.0),
                self.pos[i, 1] / (self.cfg.world_h / 2.0),
                self.vel[i, 0] / max(1e-6, self.cfg.max_speed),
                self.vel[i, 1] / max(1e-6, self.cfg.max_speed),
                math.sin(float(self.heading[i])),
                math.cos(float(self.heading[i])),
                edge / max(1e-6, max(self.cfg.world_w, self.cfg.world_h)) * self.active_mask[i],
            ])
        active_values = self.coverage_value[self.active_area_mask]
        total_active_cells = max(1, int(np.sum(self.active_area_mask)))
        exact_uncovered = int(np.sum(self.active_area_mask & (self.ever_visited == 0)))
        summary = np.array([
            float(np.sum(self.active_mask) / self.max_agents),
            float(np.mean(self.ever_visited[self.active_area_mask])) if np.any(self.active_area_mask) else 0.0,
            float(np.mean(active_values >= self.cfg.maintained_threshold)) if active_values.size > 0 else 0.0,
            float(np.mean(active_values)) if active_values.size > 0 else 0.0,
            float(np.mean(active_values < self.cfg.maintained_threshold)) if active_values.size > 0 else 0.0,
            1.0 - float(np.mean(self.ever_visited[self.active_area_mask])),
             self.step_count / max(1, self.max_steps),
             self.time_to_50 if self.time_to_50 >= 0 else self.cfg.episode_seconds,
             self.time_to_80 if self.time_to_80 >= 0 else self.cfg.episode_seconds,
             self.time_to_95 if self.time_to_95 >= 0 else self.cfg.episode_seconds,
             # ── NEW: exact uncovered count (critic can see EXACTLY how many cells remain) ──
             float(exact_uncovered) / float(total_active_cells),  # normalized [0,1]
             float(exact_uncovered) / 1000.0,                      # raw count / 1000 (gives scale info)
             1.0 if self._safety_violated else 0.0,                # safety flag for critic awareness
         ], dtype=np.float32)
        # --- Corner-specific features (two per corner) ---
        num_corners = len(self.polygon_vertices)
        corner_feats = np.zeros((num_corners * 2,), dtype=np.float32)
        if num_corners > 0:
            # Neighborhood radius (meters) around each corner to consider. Use a
            # small number of cells so the signal is local to the corner.
            radius_m = max(3 * self.cfg.grid_resolution_m, 1e-6)
            # flatten grid for vectorised distance computations
            gp = self.grid_points.reshape(-1, 2)
            cov_flat = self.coverage_value.reshape(-1)
            ever_flat = self.ever_visited.reshape(-1).astype(np.float32)
            for k, v in enumerate(self.polygon_vertices):
                dists = np.linalg.norm(gp - v[None, :], axis=1)
                mask = dists <= radius_m
                if np.any(mask):
                    local_cov = cov_flat[mask]
                    local_ever = ever_flat[mask]
                    mean_cov = float(np.mean(local_cov)) if local_cov.size > 0 else 0.0
                    frac_ever = float(np.mean(local_ever)) if local_ever.size > 0 else 0.0
                else:
                    # fallback: use nearest grid cell values
                    idx = int(np.argmin(dists))
                    mean_cov = float(cov_flat[idx])
                    frac_ever = float(ever_flat[idx])
                corner_feats[2 * k] = mean_cov
                corner_feats[2 * k + 1] = frac_ever

        global_features = np.concatenate([np.asarray(agent_feats, dtype=np.float32), summary, corner_feats], axis=0)

        # ── Contribution fairness features for critic ──
        # Per-drone recent discovery rate (normalized) + swarm Gini coefficient + CV
        n_filled_c = min(self._recent_disc_ptr, self._fairness_window)
        recent_totals_c = np.sum(self._recent_discoveries[:, :max(1, n_filled_c)], axis=1).astype(np.float32)
        max_recent_c = max(1.0, float(np.max(recent_totals_c)))
        per_drone_contrib = recent_totals_c / max_recent_c  # (max_agents,) in [0,1]
        active_contribs_c = recent_totals_c[self.active_mask > 0.5]
        if len(active_contribs_c) > 1 and float(np.sum(active_contribs_c)) > 0:
            sorted_c = np.sort(active_contribs_c)
            n_ac = len(sorted_c)
            index_c = np.arange(1, n_ac + 1)
            gini = float((2.0 * np.sum(index_c * sorted_c) - (n_ac + 1) * np.sum(sorted_c)) /
                         (n_ac * max(1.0, float(np.sum(sorted_c)))))
        else:
            gini = 0.0
        mean_c = float(np.mean(active_contribs_c)) if len(active_contribs_c) > 0 else 0.0
        std_c = float(np.std(active_contribs_c)) if len(active_contribs_c) > 1 else 0.0
        cv = std_c / max(1e-6, mean_c)
        fairness_feats = np.concatenate([per_drone_contrib, np.array([gini, min(cv, 3.0)], dtype=np.float32)])
        global_features = np.concatenate([global_features, fairness_feats], axis=0)

        return {
            'global_maps': global_maps,
            'global_features': global_features,
        }

    def get_critic_state(self) -> np.ndarray:
        obs = self.get_critic_observation()
        return np.concatenate([obs['global_maps'].reshape(-1), obs['global_features']], axis=0)

    def get_frontier_potential(self) -> np.ndarray:
        """Per-agent potential [0,1] from frontier heuristic logic.

        Each active drone is assigned an angular sector around the polygon
        centroid.  The potential is *inversely* proportional to the distance
        to the nearest uncovered / stale cell in its sector — i.e. closer
        to the frontier → higher potential.  This is used **only** as an
        auxiliary critic target; the actor never sees it.
        """
        potential = np.zeros((self.max_agents,), dtype=np.float32)
        active_idxs = np.where(self.active_mask > 0.5)[0]
        if len(active_idxs) == 0:
            return potential

        # Target cells: uncovered first, then stale
        uncovered = self.active_area_mask & (self.ever_visited == 0)
        if np.any(uncovered):
            target_pts = self.grid_points[uncovered]
        else:
            stale = self.active_area_mask & (self.coverage_value < self.cfg.maintained_threshold)
            if np.any(stale):
                target_pts = self.grid_points[stale]
            else:
                # Everything is well-covered — max potential for everyone
                potential[active_idxs] = 1.0
                return potential

        centroid = np.mean(self.polygon_vertices, axis=0)
        rel_targets = target_pts - centroid[None, :]
        target_angles = np.arctan2(rel_targets[:, 1], rel_targets[:, 0])

        drone_angles = np.arctan2(
            self.pos[active_idxs, 1] - centroid[1],
            self.pos[active_idxs, 0] - centroid[0],
        )
        order = np.argsort(drone_angles)
        ordered_active = active_idxs[order]
        sector_edges = np.linspace(-math.pi, math.pi, len(ordered_active) + 1)

        max_possible = math.sqrt(self.cfg.world_w ** 2 + self.cfg.world_h ** 2)
        for k, i in enumerate(ordered_active):
            lo, hi = sector_edges[k], sector_edges[k + 1]
            in_sector = (target_angles >= lo) & (target_angles <= hi)
            sector_pts = target_pts[in_sector] if np.any(in_sector) else target_pts
            dists = np.linalg.norm(sector_pts - self.pos[i][None, :], axis=1)
            min_dist = float(np.min(dists))
            # potential: 1 when on top of frontier, 0 when maximally far
            potential[i] = max(0.0, 1.0 - min_dist / max_possible)
        return potential

    def get_marginal_contribution(self) -> np.ndarray:
        """Per-drone marginal contribution [0,1]: how much UNIQUE uncovered
        territory does this drone have access to that no other drone is closer to?

        Uses a reachable radius (speed × horizon) to look ahead, not just the
        tiny scan cone.  Measures the fraction of uncovered cells within
        reach that are Voronoi-assigned to this drone.  A drone surrounded by
        well-covered area with no unique frontier gets ~0; a drone near a
        large exclusive frontier gets ~1.

        This teaches the critic: "this drone's position is valuable because
        it has lots of uncovered territory that only IT can efficiently reach."
        """
        result = np.zeros((self.max_agents,), dtype=np.float32)
        active_idxs = np.where(self.active_mask > 0.5)[0]
        n = len(active_idxs)
        if n < 1:
            return result

        uncovered = self.active_area_mask & (self.ever_visited == 0)
        if not np.any(uncovered):
            # Fall back to stale cells for maintenance phase
            stale = self.active_area_mask & (self.coverage_value < self.cfg.maintained_threshold)
            if not np.any(stale):
                return result
            uncovered = stale

        ys, xs = np.where(uncovered)
        total_uncovered = len(ys)
        # Subsample for speed
        step = max(1, total_uncovered // 600)
        ys, xs = ys[::step], xs[::step]
        frontier_pts = self.grid_points[ys, xs]  # (P, 2)

        # Reachable radius: how far a drone can travel in ~10 seconds
        reach_radius = self.cfg.max_speed * 10.0  # ~25m at 2.5m/s

        drone_positions = self.pos[active_idxs]
        # Distance from each frontier point to each drone: (P, n_active)
        dists = np.linalg.norm(
            frontier_pts[:, None, :] - drone_positions[None, :, :], axis=-1
        )
        # Voronoi assignment: each frontier cell belongs to its nearest drone
        nearest_drone = np.argmin(dists, axis=1)  # (P,)
        nearest_dist = dists[np.arange(len(dists)), nearest_drone]  # (P,)

        for local_idx, drone_idx in enumerate(active_idxs):
            # Cells assigned to this drone AND within reachable distance
            owned = (nearest_drone == local_idx) & (nearest_dist <= reach_radius)
            n_owned = int(np.sum(owned))

            # Also count owned cells that are CLOSE (within 2× scan range)
            # — these are immediately actionable
            close_owned = owned & (dists[:, local_idx] <= self.cfg.scan_range_m * 2.0)
            n_close = int(np.sum(close_owned))

            # Blend: nearby uncovered cells are more valuable
            # Normalize by what an "ideal" drone would own (1/N of total reachable)
            ideal_owned = max(1, len(ys)) / max(1, n)
            territory_score = min(1.0, n_owned / max(1.0, ideal_owned))
            proximity_score = min(1.0, n_close / max(1.0, ideal_owned * 0.3))

            result[drone_idx] = 0.5 * territory_score + 0.5 * proximity_score

        return result

    def get_territorial_fraction(self) -> np.ndarray:
        """Per-drone territorial fraction [0,1]: what fraction of UNCOVERED
        cells are closest to THIS drone (Voronoi ownership).

        Ideally each drone owns ~1/N of the uncovered cells.  The critic
        learns to predict this, and the value function naturally rewards
        positions where the drone has a large territory of unexplored cells.
        """
        result = np.zeros((self.max_agents,), dtype=np.float32)
        active_idxs = np.where(self.active_mask > 0.5)[0]
        n = len(active_idxs)
        if n < 1:
            return result

        uncovered = self.active_area_mask & (self.ever_visited == 0)
        if not np.any(uncovered):
            # No uncovered cells — equal ownership
            result[active_idxs] = 1.0 / n
            return result

        ys, xs = np.where(uncovered)
        total_uncovered = len(ys)
        # Subsample for speed
        step = max(1, total_uncovered // 500)
        ys, xs = ys[::step], xs[::step]
        frontier_pts = self.grid_points[ys, xs]

        drone_positions = self.pos[active_idxs]
        dists = np.linalg.norm(
            frontier_pts[:, None, :] - drone_positions[None, :, :], axis=-1
        )
        assignments = np.argmin(dists, axis=1)

        for local_idx, drone_idx in enumerate(active_idxs):
            n_owned = int(np.sum(assignments == local_idx))
            result[drone_idx] = n_owned / max(1, len(ys))
        return result

    def get_guidance_vector(self) -> np.ndarray:
        """Per-drone guidance vector (2D unit direction): where this drone SHOULD
        move, computed from multiple signals.  CRITIC-ONLY — actor never sees this.

        Combines three forces:
          1. FRONTIER PULL   — Voronoi-assigned uncovered centroid direction
          2. DRONE REPULSION — push away from nearby drones to spread out
          3. CORNER PULL     — attract toward under-covered polygon corners (late-game)

        The blending weights shift over the episode:
          - Early (coverage < 80%):  frontier=0.7, repulsion=0.3, corner=0.0
          - Mid   (80-95%):          frontier=0.5, repulsion=0.2, corner=0.3
          - Late  (>95%):            frontier=0.3, repulsion=0.1, corner=0.6

        Returns: (max_agents, 2) — unit direction vectors (or zero if no guidance).
        """
        result = np.zeros((self.max_agents, 2), dtype=np.float32)
        active_idxs = np.where(self.active_mask > 0.5)[0]
        n = len(active_idxs)
        if n < 1:
            return result

        es = self.last_ever_seen_fraction

        # ── Phase-dependent blending weights ──
        if es < 0.80:
            w_frontier, w_repulsion, w_corner = 0.7, 0.3, 0.0
        elif es < 0.95:
            w_frontier, w_repulsion, w_corner = 0.5, 0.2, 0.3
        else:
            w_frontier, w_repulsion, w_corner = 0.3, 0.1, 0.6

        # ── 1. Frontier pull: direction toward Voronoi-assigned uncovered cells ──
        frontier_dirs = np.zeros((self.max_agents, 2), dtype=np.float32)
        uncovered = self.active_area_mask & (self.ever_visited == 0)
        if not np.any(uncovered):
            # Fall back to stale cells for maintenance
            stale = self.active_area_mask & (self.coverage_value < self.cfg.maintained_threshold)
            if np.any(stale):
                uncovered = stale
            else:
                w_frontier = 0.0  # nothing to explore

        if np.any(uncovered):
            ys, xs = np.where(uncovered)
            step = max(1, len(ys) // 500)
            frontier_pts = self.grid_points[ys[::step], xs[::step]]
            drone_positions = self.pos[active_idxs]
            dists = np.linalg.norm(
                frontier_pts[:, None, :] - drone_positions[None, :, :], axis=-1
            )
            assignments = np.argmin(dists, axis=1)  # which drone owns each cell

            for local_idx, drone_idx in enumerate(active_idxs):
                owned_mask = assignments == local_idx
                if np.any(owned_mask):
                    owned_pts = frontier_pts[owned_mask]
                    # Weight by inverse distance — prefer closer cells
                    d = np.linalg.norm(owned_pts - self.pos[drone_idx], axis=1)
                    weights = 1.0 / (d + 0.5)
                    waypoint = np.average(owned_pts, axis=0, weights=weights)
                else:
                    # No owned cells — go to nearest frontier
                    nearest = np.argmin(dists[:, local_idx])
                    waypoint = frontier_pts[nearest]
                delta = waypoint - self.pos[drone_idx]
                norm = np.linalg.norm(delta)
                if norm > 1e-3:
                    frontier_dirs[drone_idx] = delta / norm

        # ── 2. Drone repulsion: push away from nearby drones ──
        repulsion_dirs = np.zeros((self.max_agents, 2), dtype=np.float32)
        if n >= 2:
            area = float(np.sum(self.active_area_mask)) * self.cfg.grid_resolution_m ** 2
            ideal_dist = math.sqrt(area / max(n, 1))
            for local_idx, i in enumerate(active_idxs):
                rep = np.zeros(2, dtype=np.float32)
                for j in active_idxs:
                    if i == j:
                        continue
                    diff = self.pos[i] - self.pos[j]
                    d = np.linalg.norm(diff)
                    if d < ideal_dist and d > 1e-6:
                        # Strength: inversely proportional to distance squared
                        strength = ((ideal_dist - d) / ideal_dist) ** 2
                        rep += (diff / d) * strength
                rep_norm = np.linalg.norm(rep)
                if rep_norm > 1e-6:
                    repulsion_dirs[i] = rep / rep_norm

        # ── 3. Corner pull: attract toward under-covered polygon corners ──
        corner_dirs = np.zeros((self.max_agents, 2), dtype=np.float32)
        if w_corner > 0 and len(self.polygon_vertices) > 0:
            # Find corners with lowest local coverage
            corner_scores = np.zeros(len(self.polygon_vertices), dtype=np.float32)
            radius_cells = max(1, int(self.cfg.corner_scan_distance / self.cfg.grid_resolution_m))
            for k, v in enumerate(self.polygon_vertices):
                ix_c = int(np.clip(np.round((v[0] + self.cfg.world_w / 2.0) / self.cfg.grid_resolution_m - 0.5), 0, self.grid_w - 1))
                iy_c = int(np.clip(np.round((v[1] + self.cfg.world_h / 2.0) / self.cfg.grid_resolution_m - 0.5), 0, self.grid_h - 1))
                y_lo = max(0, iy_c - radius_cells)
                y_hi = min(self.grid_h, iy_c + radius_cells + 1)
                x_lo = max(0, ix_c - radius_cells)
                x_hi = min(self.grid_w, ix_c + radius_cells + 1)
                patch_active = self.active_area_mask[y_lo:y_hi, x_lo:x_hi]
                total = np.sum(patch_active)
                if total > 0:
                    patch_ever = self.ever_visited[y_lo:y_hi, x_lo:x_hi]
                    corner_scores[k] = 1.0 - float(np.sum(patch_ever[patch_active])) / float(total)
                # corner_scores[k] = fraction of cells near corner that are UNCOVERED

            for local_idx, drone_idx in enumerate(active_idxs):
                # Each drone is attracted to the worst-covered corner that is
                # closest to it (weighted by both distance and uncovered fraction)
                deltas = self.polygon_vertices - self.pos[drone_idx]
                dists_to_corners = np.linalg.norm(deltas, axis=1)
                # Score: high uncoverage × close distance = attractive
                attraction = corner_scores / (dists_to_corners + 1.0)
                best_corner = np.argmax(attraction)
                if corner_scores[best_corner] > 0.01:  # only if there's something uncovered
                    delta = deltas[best_corner]
                    norm = np.linalg.norm(delta)
                    if norm > 1e-3:
                        corner_dirs[drone_idx] = delta / norm

        # ── Blend all three forces ──
        for i in active_idxs:
            combined = (w_frontier * frontier_dirs[i]
                        + w_repulsion * repulsion_dirs[i]
                        + w_corner * corner_dirs[i])
            norm = np.linalg.norm(combined)
            if norm > 1e-6:
                result[i] = combined / norm

        return result

    def _scan_efficiency_score(self) -> float:
        """Single score [0, 100] measuring mission success.

        HARD SAFETY GATE: if ANY collision (drone-drone or wall) occurred
        during the episode, the score is EXACTLY 0.0.

        Otherwise:
          40%  ever-seen fraction — did you cover the area?
          25%  maintained fraction — are you keeping it covered?
          20%  speed bonus — how quickly did you reach 80%/95% coverage?
          15%  safety — low overlap + away from walls
        """
        # ── HARD GATE: safety violations → score = 0 ──
        if self._safety_violated:
            return 0.0

        ever_seen = float(np.mean(self.ever_visited[self.active_area_mask]))
        active_values = self.coverage_value[self.active_area_mask]
        maintained = float(np.mean(active_values >= self.cfg.maintained_threshold))

        # Speed: reward reaching milestones fast (normalised by episode length)
        ep_len = self.cfg.episode_seconds
        t80 = self.time_to_80 if self.time_to_80 >= 0 else ep_len
        t95 = self.time_to_95 if self.time_to_95 >= 0 else ep_len
        speed_80 = max(0.0, 1.0 - t80 / ep_len)
        speed_95 = max(0.0, 1.0 - t95 / ep_len)
        speed = 0.5 * speed_80 + 0.5 * speed_95

        # Safety: low overlap
        overlap = float(np.mean(self._overlap_ratio()[self.active_mask > 0.5])) if np.any(self.active_mask > 0.5) else 0.0
        safety = max(0.0, 1.0 - overlap)

        return float(100.0 * (0.40 * ever_seen
                              + 0.25 * maintained
                              + 0.20 * speed
                              + 0.15 * safety))

    def _build_info(self, rewards: np.ndarray):
        active_values = self.coverage_value[self.active_area_mask]
        active_agents = max(1.0, float(np.sum(self.active_mask)))
        total_active_cells = max(1, int(np.sum(self.active_area_mask)))
        uncovered_cells = int(np.sum(self.active_area_mask & (self.ever_visited == 0)))
        return {
            'reward_mean': float(np.sum(rewards) / active_agents),
            'coverage_mean': float(np.mean(active_values)),
            'ever_seen_fraction': float(np.mean(self.ever_visited[self.active_area_mask])),
            'percent_covered_at_least_once': 100.0 * float(np.mean(self.ever_visited[self.active_area_mask])),
            'maintained_fraction': float(np.mean(active_values >= self.cfg.maintained_threshold)),
            'uncovered_fraction': 1.0 - float(np.mean(self.ever_visited[self.active_area_mask])),
            'uncovered_cells': uncovered_cells,
            'total_active_cells': total_active_cells,
            'stale_fraction': float(np.mean(active_values < self.cfg.maintained_threshold)),
            'time_to_50_coverage': float(self.time_to_50),
            'time_to_80_coverage': float(self.time_to_80),
            'time_to_95_coverage': float(self.time_to_95),
            'episode_progress': self.step_count / max(1, self.max_steps),
            # ── SAFETY hard gate tracking ──
            'safety_violated': self._safety_violated,
            'episode_collision_count': self._episode_collision_count,
            'episode_wall_collision_count': self._episode_wall_collision_count,
            'maint_quality': self.get_maintenance_quality_score(),
        }

    def check_mission_feasibility(self) -> dict:
        """Check whether the mission is physically feasible given speed, time,
        area, number of drones, and scan parameters.

        Returns a dict with feasibility metrics and a boolean 'feasible' flag.
        """
        area_m2 = float(polygon_area(self.polygon_vertices))
        # Coverable area excludes dead-zone near walls
        coverable_cells = int(np.sum(self.active_area_mask))
        coverable_area_m2 = coverable_cells * self.cfg.grid_resolution_m ** 2

        n_drones = int(np.sum(self.active_mask > 0.5))
        episode_time = self.cfg.episode_seconds

        # Scan swath width: at max speed, how wide a strip does one pass cover?
        scan_half_angle = math.radians(self.cfg.scan_fov_deg) / 2.0
        swath_width_m = 2.0 * self.cfg.scan_range_m * math.sin(scan_half_angle)

        # Area one drone can cover in one pass (lawnmower pattern, no overlap)
        strip_area_per_second = self.cfg.max_speed * swath_width_m
        total_coverable_by_fleet = strip_area_per_second * n_drones * episode_time

        # Coverage ratio: how many times the fleet can theoretically sweep the area
        coverage_ratio = total_coverable_by_fleet / max(coverable_area_m2, 1.0)

        # Time to first-cover everything (single pass, no revisits)
        time_to_first_cover = coverable_area_m2 / max(strip_area_per_second * n_drones, 1e-6)

        # Maintenance: after first cover, how often must each cell be revisited?
        decay_period = self.cfg.decay_reset_seconds
        revisit_rate_needed = coverable_area_m2 / decay_period  # m²/s that must be refreshed
        revisit_capacity = strip_area_per_second * n_drones
        maintenance_ratio = revisit_capacity / max(revisit_rate_needed, 1e-6)

        # Step distance vs scan width — will there be gaps between steps?
        step_distance = self.cfg.max_speed * self.dt
        gap_free = step_distance <= swath_width_m

        feasible = (
            coverage_ratio >= 1.5  # at least 1.5× theoretical capacity
            and time_to_first_cover <= episode_time * 0.7  # first cover in 70% of episode
            and maintenance_ratio >= 1.0  # can sustain coverage
            and gap_free  # no gaps at max speed
        )

        return {
            'feasible': feasible,
            'polygon_area_m2': area_m2,
            'coverable_area_m2': coverable_area_m2,
            'n_drones': n_drones,
            'episode_seconds': episode_time,
            'swath_width_m': swath_width_m,
            'step_distance_m': step_distance,
            'gap_free': gap_free,
            'strip_area_per_second_per_drone': strip_area_per_second,
            'coverage_ratio': coverage_ratio,
            'time_to_first_cover_s': time_to_first_cover,
            'maintenance_ratio': maintenance_ratio,
        }

    def render_state(self):
        return {
            'polygon_vertices': self.polygon_vertices.copy(),
            'coverage_value': self.coverage_value.copy(),
            'ever_visited': self.ever_visited.copy(),
            'active_area': self.active_area_mask.astype(np.uint8).copy(),
            'uncovered_mask': (self.active_area_mask & (self.ever_visited == 0)).astype(np.uint8).copy(),
            'stale_mask': ((self.coverage_value < self.cfg.maintained_threshold) & self.active_area_mask).astype(np.uint8).copy(),
            'positions': self.pos.copy(),
            'previous_positions': self.prev_pos.copy(),
            'velocities': self.vel.copy(),
            'headings': self.heading.copy(),
            'active_mask': self.active_mask.copy(),
            'grid_resolution_m': np.array([self.cfg.grid_resolution_m], dtype=np.float32),
        }
