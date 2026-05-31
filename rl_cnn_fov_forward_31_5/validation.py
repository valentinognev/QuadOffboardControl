# validation.py — Comprehensive coverage validation for multi-drone swarm
import numpy as np
import time
from collections import defaultdict
import os
import shutil
from pathlib import Path
import json

from polygon_utils import distance_to_polygon_edge, point_in_polygon


class CoverageValidator:
    """Validates and measures efficiency of multi-drone area coverage."""

    def __init__(self, grid_shape, n_drones=5, grid_resolution=0.5,
                 drone_safety_radius=2.0, wall_safety_margin=1.5,
                 decay_rate=0.97, polygon_vertices=None,
                 world_w=40.0, world_h=40.0, max_steps=None):
        self.grid_shape = grid_shape  # (rows, cols)
        self.n_drones = n_drones
        self.grid_res = grid_resolution
        self.drone_safety_radius = drone_safety_radius
        self.wall_safety_margin = wall_safety_margin
        self.decay_rate = decay_rate
        self.polygon_vertices = polygon_vertices  # Nx2 array or None
        self.world_w = world_w
        self.world_h = world_h
        self.max_steps = max_steps
        self.total_cells = grid_shape[0] * grid_shape[1]
        self._active_mask = None  # set externally after creation
        self.reset()

    @property
    def active_cells(self):
        """Number of cells inside the active polygon area (excluding margin/outside)."""
        if self._active_mask is not None:
            return int(np.sum(self._active_mask))
        return self.total_cells

    def reset(self):
        self.visited = np.zeros(self.grid_shape, dtype=np.int32)
        self.first_visitor = -np.ones(self.grid_shape, dtype=np.int16)
        self.drone_paths = defaultdict(list)        # grid coords
        self.drone_world_paths = defaultdict(list)  # world coords
        self.drone_headings = defaultdict(list)
        self.step_count = 0
        self.coverage_over_time = []
        self.maintained_over_time = []
        self.overlap_count = 0
        self.collision_count = 0
        self.collision_events = []
        self.wall_collision_count = 0       # drone OUTSIDE polygon (hard fail)
        self.wall_collision_events = []
        self.wall_violation_count = 0       # drone inside but too close (warning only)
        self.wall_violation_events = []
        self.stagnation_events = defaultdict(int)
        self.start_time = time.time()

        # Coverage decay tracking
        self.coverage_age = np.zeros(self.grid_shape, dtype=np.float32)

        # Per-step metrics for curves
        self.new_cells_per_step = []
        self.spread_over_time = []
        self.heading_diversity_over_time = []

    # ─────���────────────────────────────────────────────────────
    #  RECORDING
    # ──────────────────────────────────────────────────────────
    def record_step(self, drone_positions, drone_world_positions=None,
                    drone_headings=None):
        """
        Record one timestep.
        - drone_positions:       list of (row, col) grid indices
        - drone_world_positions: list of (x, y) in metres (optional, for distance)
        - drone_headings:        list of heading angles in radians (optional)
        """
        self.step_count += 1
        new_cells = 0

        # Age all cells
        self.coverage_age += 1

        for drone_id, pos in enumerate(drone_positions):
            r, c = int(pos[0]), int(pos[1])
            if 0 <= r < self.grid_shape[0] and 0 <= c < self.grid_shape[1]:
                if self.visited[r, c] > 0:
                    self.overlap_count += 1
                else:
                    new_cells += 1
                    self.first_visitor[r, c] = drone_id
                self.visited[r, c] += 1
                self.coverage_age[r, c] = 0  # refresh
                self.drone_paths[drone_id].append((r, c))

            if drone_world_positions is not None:
                self.drone_world_paths[drone_id].append(
                    np.array(drone_world_positions[drone_id], dtype=np.float32))

            if drone_headings is not None:
                self.drone_headings[drone_id].append(float(drone_headings[drone_id]))

        self.new_cells_per_step.append(new_cells)

        # Coverage %
        if self._active_mask is not None:
            covered = np.count_nonzero(self.visited[self._active_mask])
        else:
            covered = np.count_nonzero(self.visited)
        self.coverage_over_time.append(covered / self.active_cells * 100)

        # Maintained coverage (cells refreshed within decay window)
        maintained = np.sum(self.coverage_age < (1.0 / (1.0 - self.decay_rate)))
        self.maintained_over_time.append(maintained / self.active_cells * 100)

        # ── Collision detection ──
        # Only check ACTIVE drones (inactive drones sit at (0,0) and would
        # create false positives or miss real collisions)
        if drone_world_positions is not None:
            wpos = np.array(drone_world_positions)
            n_drones_check = min(len(wpos), self.n_drones)
            for i in range(n_drones_check):
                for j in range(i + 1, n_drones_check):
                    dist = np.linalg.norm(wpos[i] - wpos[j])
                    if dist < self.drone_safety_radius:
                        self.collision_count += 1
                        self.collision_events.append(
                            (self.step_count, i, j, round(float(dist), 3)))

        # ── Wall / boundary detection ──
        # Two severity levels:
        #   COLLISION: drone is OUTSIDE the polygon → hard safety fail
        #   VIOLATION: drone is inside but closer than wall_safety_margin → warning only
        if drone_world_positions is not None and self.polygon_vertices is not None:
            for drone_id, wp in enumerate(drone_world_positions):
                pt = np.array(wp, dtype=np.float64)
                inside = point_in_polygon(pt, self.polygon_vertices)
                d = distance_to_polygon_edge(pt, self.polygon_vertices)
                if not inside:
                    # Drone is OUTSIDE the polygon — WALL COLLISION (hard fail)
                    self.wall_collision_count += 1
                    self.wall_collision_events.append(
                        (self.step_count, drone_id, round(float(-d), 3)))
                elif d < self.wall_safety_margin:
                    # Drone is inside but too close — WARNING only
                    self.wall_violation_count += 1
                    self.wall_violation_events.append(
                        (self.step_count, drone_id, round(float(d), 3)))

        # ── Stagnation detection ──
        stag_window = 20
        for drone_id in range(self.n_drones):
            path = self.drone_paths[drone_id]
            if len(path) >= stag_window:
                recent = path[-stag_window:]
                if len(set(recent)) <= 2:
                    self.stagnation_events[drone_id] += 1

        # ── Spatial spread ──
        if drone_world_positions is not None and len(drone_world_positions) >= 2:
            wpos = np.array(drone_world_positions)
            dists = []
            for i in range(len(wpos)):
                for j in range(i + 1, len(wpos)):
                    dists.append(np.linalg.norm(wpos[i] - wpos[j]))
            self.spread_over_time.append(float(np.mean(dists)))
        else:
            self.spread_over_time.append(0.0)

        # ── Heading diversity ──
        if drone_headings is not None and len(drone_headings) >= 2:
            h = np.array(drone_headings)
            S = np.mean(np.sin(h))
            C = np.mean(np.cos(h))
            R = np.sqrt(S**2 + C**2)
            circ_var = 1 - R  # 0 = all same, 1 = max diversity
            self.heading_diversity_over_time.append(float(circ_var))
        else:
            self.heading_diversity_over_time.append(0.0)

    # ──────────────────────────────────────────────────────────
    #  FOV SYNC
    # ──────────────────────────────────────────────────────────
    def sync_coverage_from_fov(self, env_ever_visited, drone_positions):
        """Sync visited grid with the env's FOV-based coverage AND update first_visitor.

        Finds newly-covered cells (present in env but not yet in validator) and
        assigns each to the NEAREST drone so that first_visitor / unique_contribution
        reflect the true FOV-based discovery.

        CRITICAL: Also updates coverage_over_time and new_cells_per_step to reflect
        the full FOV-based coverage. Without this, coverage metrics would only count
        the single cell each drone stands on (massively undercounting).

        Args:
            env_ever_visited: np.ndarray – boolean or int array from env
            drone_positions:  list of (row, col) grid positions for each drone
        """
        env_mask = env_ever_visited.astype(bool)
        already = self.visited > 0
        new_cells_mask = env_mask & ~already

        # Count FOV-based new cells discovered this step (replace single-cell count)
        fov_new_cells = int(np.sum(new_cells_mask))

        if np.any(new_cells_mask):
            new_coords = np.argwhere(new_cells_mask)
            drone_rc = np.array(drone_positions, dtype=np.float32)
            for rc in new_coords:
                dists = np.linalg.norm(drone_rc - rc.astype(np.float32), axis=1)
                nearest = int(np.argmin(dists))
                if self.first_visitor[rc[0], rc[1]] == -1:
                    self.first_visitor[rc[0], rc[1]] = nearest

        self.visited = np.maximum(self.visited, env_ever_visited.astype(np.int32))

        # ── Fix coverage_over_time to reflect FOV-based coverage ──
        # record_step computed coverage BEFORE this sync, so the last entry
        # only counted single-cell drone positions. Recompute with synced data.
        if self._active_mask is not None:
            covered = np.count_nonzero(self.visited[self._active_mask])
        else:
            covered = np.count_nonzero(self.visited)
        if self.coverage_over_time:
            self.coverage_over_time[-1] = covered / self.active_cells * 100
        if self.new_cells_per_step:
            self.new_cells_per_step[-1] = fov_new_cells

    # ──────────────────────────────────────────────────────────
    #  METRICS
    # ──────────────────────────────────────────────────────────
    def get_metrics(self):
        if self._active_mask is not None:
            covered = np.count_nonzero(self.visited[self._active_mask])
        else:
            covered = np.count_nonzero(self.visited)
        coverage_pct = covered / self.active_cells * 100
        total_visits = int(self.visited.sum())
        overlap_ratio = (total_visits - covered) / max(total_visits, 1) * 100
        elapsed = time.time() - self.start_time

        # ── Thresholds ──
        thresholds = {t: None for t in [50, 75, 90, 95, 98, 100]}
        for step, cov in enumerate(self.coverage_over_time):
            for t in thresholds:
                if thresholds[t] is None and cov >= t:
                    thresholds[t] = step + 1

        # ── Per-drone workload ──
        drone_cell_counts = [len(self.drone_paths[d]) for d in range(self.n_drones)]
        fairness_std = float(np.std(drone_cell_counts))

        # ── Per-drone unique contribution ──
        unique_contrib = []
        for d in range(self.n_drones):
            unique_contrib.append(int(np.sum(self.first_visitor == d)))

        # ── Total distance flown per drone ──
        drone_distances = []
        for d in range(self.n_drones):
            wpath = self.drone_world_paths.get(d, [])
            if len(wpath) >= 2:
                pts = np.array(wpath)
                dist = float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))
            else:
                pts_grid = self.drone_paths.get(d, [])
                if len(pts_grid) >= 2:
                    arr = np.array(pts_grid, dtype=np.float32) * self.grid_res
                    dist = float(np.sum(np.linalg.norm(np.diff(arr, axis=0), axis=1)))
                else:
                    dist = 0.0
            drone_distances.append(round(dist, 2))

        total_distance = sum(drone_distances)
        distance_per_cell = round(total_distance / max(covered, 1), 3)

        # ── Total heading change (energy proxy for turns) ──
        total_turn_rad = 0.0
        for d in range(self.n_drones):
            hdgs = self.drone_headings.get(d, [])
            if len(hdgs) >= 2:
                h = np.array(hdgs)
                dh = np.abs(np.diff(h))
                dh = np.minimum(dh, 2 * np.pi - dh)
                total_turn_rad += float(np.sum(dh))
        energy_estimate = round(total_distance + 0.5 * total_turn_rad, 2)

        # ── Circling detection ──
        circling_events = 0
        for d in range(self.n_drones):
            path = self.drone_paths[d]
            if len(path) >= 40:
                for i in range(0, len(path) - 39, 10):
                    window = path[i:i + 40]
                    unique_cells = len(set(window))
                    if unique_cells < 8:
                        circling_events += 1

        # ── Coverage rate (new cells/step) over windows ──
        rates = self.new_cells_per_step
        if len(rates) >= 100:
            early_rate = np.mean(rates[:len(rates)//3])
            late_rate = np.mean(rates[2*len(rates)//3:])
        else:
            early_rate = np.mean(rates) if rates else 0
            late_rate = early_rate
        coverage_rate_slowdown = round(
            (early_rate - late_rate) / max(early_rate, 1e-6), 3)

        # ── Average spread & heading diversity ──
        avg_spread = round(float(np.mean(self.spread_over_time)), 2) if self.spread_over_time else 0.0
        avg_heading_div = round(float(np.mean(self.heading_diversity_over_time)), 3) if self.heading_diversity_over_time else 0.0

        # ── Maintained coverage (last value) ──
        maintained_pct = round(self.maintained_over_time[-1], 2) if self.maintained_over_time else 0.0

        return {
            # ── Core coverage ──
            "coverage_pct": round(coverage_pct, 2),
            "cells_covered": covered,
            "total_cells": self.active_cells,
            "maintained_coverage_pct": maintained_pct,
            "total_steps": self.step_count,
            "steps_to_threshold": thresholds,

            # ── Efficiency ──
            "efficiency_score": round(covered / max(self.step_count * self.n_drones, 1), 4),
            "overlap_ratio_pct": round(overlap_ratio, 2),
            "distance_per_new_cell_m": distance_per_cell,
            "coverage_rate_slowdown": coverage_rate_slowdown,

            # ── Safety ──
            "collision_count": self.collision_count,
            "wall_collision_count": self.wall_collision_count,
            "wall_violation_count": self.wall_violation_count,

            # ── Path quality ──
            "total_distance_m": round(total_distance, 2),
            "drone_distances_m": drone_distances,
            "energy_estimate": energy_estimate,
            "circling_events": circling_events,
            "stagnation_events": dict(self.stagnation_events),

            # ── Coordination ──
            "drone_workload": drone_cell_counts,
            "workload_std": round(fairness_std, 2),
            "unique_contribution": unique_contrib,
            "avg_inter_drone_spread_m": avg_spread,
            "avg_heading_diversity": avg_heading_div,

            # ── Timing ──
            "elapsed_sec": round(elapsed, 2),
        }

    def _grade(self, m):
        """Produce a simple grade string from metrics.

        Heuristic scoring (0-100):
        - coverage_pct (0..100) weighted 50%
        - efficiency_score (normalized) weighted 10%
        - overlap (penalty) weighted 10%
        - collisions (penalty) weighted 20%
        - maintained_coverage_pct weighted 10%
        """
        # Base from coverage and maintained coverage
        cov = float(m.get('coverage_pct', 0.0))
        maint = float(m.get('maintained_coverage_pct', 0.0))
        overlap = float(m.get('overlap_ratio_pct', 100.0))
        coll = int(m.get('collision_count', 0))
        wall_coll = int(m.get('wall_collision_count', 0))
        wall_warn = int(m.get('wall_violation_count', 0))
        eff = float(m.get('efficiency_score', 0.0))

        # ── HARD SAFETY GATE ──
        # Drone-drone collisions OR wall collisions (outside polygon) → score is exactly 0
        # Wall proximity violations (inside but close) are warnings only, do NOT zero the score
        if coll > 0 or wall_coll > 0:
            return "F (0.0) [SAFETY VIOLATION: {} drone collisions, {} wall collisions]".format(coll, wall_coll)

        # Normalize efficiency roughly: assume reasonable eff in [0,1]
        eff_score = min(eff * 100.0, 100.0)

        score = 0.5 * cov + 0.1 * eff_score + 0.1 * (100.0 - overlap) + 0.2 * 100 + 0.1 * maint
        score = max(0.0, min(100.0, score))

        if score >= 90:
            grade = 'A'
        elif score >= 80:
            grade = 'B'
        elif score >= 70:
            grade = 'C'
        elif score >= 60:
            grade = 'D'
        else:
            grade = 'F'
        return f"{grade} ({score:.1f})"

    def print_report(self):
        m = self.get_metrics()
        W = 70
        print("=" * W)
        print("         COVERAGE VALIDATION REPORT")
        print("=" * W)

        print("\n┌─── COVERAGE ───────────────────────────────────────────────┐")
        print(f"│  Coverage:            {m['coverage_pct']}% ({m['cells_covered']}/{m['total_cells']} cells)")
        print(f"│  Maintained coverage: {m['maintained_coverage_pct']}%")
        print(f"│  Steps to thresholds:")
        for t, s in m['steps_to_threshold'].items():
            status = f"{s:>6} steps" if s else "NOT REACHED"
            print(f"│    {t:>3}%  {status}")
        print(f"└{'─' * (W-2)}┘")

        print(f"\n┌─── EFFICIENCY ─────────────────────────────────────────────┐")
        print(f"│  Efficiency score:    {m['efficiency_score']}")
        print(f"│  Overlap ratio:       {m['overlap_ratio_pct']}%")
        print(f"│  Distance / new cell: {m['distance_per_new_cell_m']} m")
        print(f"│  Rate slowdown:       {m['coverage_rate_slowdown']}")
        print(f"└{'─' * (W-2)}┘")

        print(f"\n┌─── SAFETY ─────────────────────────────────────────────────┐")
        print(f"│  Drone collisions:    {m['collision_count']}")
        if self.collision_events:
            shown = self.collision_events[:10]
            for evt in shown:
                print(f"│    step={evt[0]:>4}  D{evt[1]}↔D{evt[2]}  dist={evt[3]}m")
            if len(self.collision_events) > 10:
                print(f"│    ... and {len(self.collision_events) - 10} more")
        print(f"│  Wall collisions:     {m['wall_collision_count']}  (outside polygon = HARD FAIL)")
        if self.wall_collision_events:
            shown = self.wall_collision_events[:10]
            for evt in shown:
                print(f"│    step={evt[0]:>4}  D{evt[1]}  outside by {abs(evt[2])}m")
            if len(self.wall_collision_events) > 10:
                print(f"│    ... and {len(self.wall_collision_events) - 10} more")
        print(f"│  Wall proximity:      {m['wall_violation_count']}  (inside but close = warning)")
        if self.wall_violation_events:
            shown = self.wall_violation_events[:5]
            for evt in shown:
                print(f"│    step={evt[0]:>4}  D{evt[1]}  dist={evt[2]}m")
            if len(self.wall_violation_events) > 5:
                print(f"│    ... and {len(self.wall_violation_events) - 5} more")
        print(f"└{'─' * (W-2)}┘")

        print(f"\n┌─── PATH QUALITY ───────────────────────────────────────────┐")
        print(f"│  Total distance:      {m['total_distance_m']} m")
        print(f"│  Per-drone distance:  {m['drone_distances_m']}")
        print(f"│  Energy estimate:     {m['energy_estimate']}")
        print(f"│  Circling events:     {m['circling_events']}")
        print(f"│  Stagnation events:   {m['stagnation_events']}")
        print(f"└{'─' * (W-2)}┘")

        print(f"\n┌─── COORDINATION ───────────────────────────────────────────┐")
        print(f"│  Workload / drone:    {m['drone_workload']}")
        print(f"│  Workload std:        {m['workload_std']}")
        print(f"│  Unique contribution: {m['unique_contribution']}")
        print(f"│  Avg spread:          {m['avg_inter_drone_spread_m']} m")
        print(f"│  Heading diversity:   {m['avg_heading_diversity']}")
        print(f"└{'─' * (W-2)}┘")

        print(f"\n┌─── TIMING ───────────────────────────────────��─────────────┐")
        max_s = f" / {self.max_steps}" if self.max_steps else ""
        print(f"│  Steps completed:     {m['total_steps']}{max_s}")
        print(f"│  Wall-clock time:     {m['elapsed_sec']} s")
        print(f"└{'─' * (W-2)}┘")

        # ── GRADE ──
        grade = self._grade(m)
        print(f"\n{'=' * W}")
        print(f"  OVERALL GRADE: {grade}")
        print(f"{'=' * W}")
        print(f"\n  (Run print_metric_summary() for detailed explanation of each metric)")

    def print_metric_summary(self):
        """Print a separate detailed summary explaining every metric and parameter."""
        W = 70
        print("\n" + "=" * W)
        print("         METRIC & PARAMETER REFERENCE GUIDE")
        print("=" * W)

        print("""
┌─── COVERAGE METRICS ──────────────────────────────────────┐
│                                                            │
│  coverage_pct                                              │
│    Percentage of active-area cells visited at least once.  │
│    This is the PRIMARY success metric. 100% = full cover.  │
│                                                            │
│  maintained_coverage_pct                                   │
│    Percentage of cells refreshed within the decay window.  ���
│    Decay window ≈ 1/(1-decay_rate) steps.                  │
│    Important for persistent surveillance: stale cells      │
│    represent gaps in monitoring.                           │
│                                                            │
│  steps_to_threshold                                        │
│    Number of steps to reach each milestone (50-100%).      │
│    Faster convergence = better exploration strategy.        │
│                                                            │
└────────────────────────────────────────────────────────────┘

┌─── EFFICIENCY METRICS ────────────────────────────────────┐
│                                                            │
│  efficiency_score                                          │
│    = cells_covered / (total_steps × n_drones)              │
│    Measures how productively each drone-step was used.     │
│    1.0 = perfect (every step discovers a new cell).        │
│    Typical good performance: 0.3 - 0.6                     │
│                                                            │
│  overlap_ratio_pct                                         │
│    Percentage of total cell-visits that were redundant     │
│    (the cell had already been visited before).             │
│    Lower = less wasted effort. 0% = no revisits at all.    │
│                                                            │
│  distance_per_new_cell_m                                   │
│    Average distance (meters) flown to discover one new     │
│    cell. Lower = more efficient paths.                     │
│                                                            │
│  coverage_rate_slowdown                                    │
│    Ratio of how much exploration slowed between early and  │
│    late phases. 0 = constant rate; 1 = exploration fully   │
│    stopped. High values = late-game inefficiency.          │
│                                                            │
└────────────────────────────────────────────────────────────┘

┌─── SAFETY METRICS ────────────────────────────────────────┐
│                                                            │
│  collision_count                                           │
│    Total step×pair instances where two drones were closer  │
│    than drone_safety_radius. Each step counted separately. │
│    Target: 0 (zero collisions).                            │
│                                                            │
│  wall_violation_count                                      │
│    Total step×drone instances where a drone exited the     │
│    polygon boundary. Target: 0.                            │
│                                                            │
└────────────────────────────────────────────────────────────┘

┌─── PATH QUALITY METRICS ──────────────────────────────────┐
│                                                            │
│  total_distance_m                                          │
│    Sum of distances flown by all drones (meters).          │
│                                                            │
│  drone_distances_m                                         │
│    Distance per drone. Should be roughly equal for         │
│    balanced energy consumption.                            │
│                                                            │
│  energy_estimate                                           │
│    Approximate energy = distance + 0.5 × turning_radians. │
│    Penalizes excessive turning which costs battery.        │
│                                                            │
│  circling_events                                           │
│    Number of 40-step windows where a drone visited fewer   │
│    than 8 unique cells (indicates looping behavior).       │
│                                                            │
│  stagnation_events                                         │
│    Per-drone count of 20-step windows with ≤2 unique       │
│    cells visited (drone is completely stuck).              │
│                                                            │
└────────────────────────────────────────────────────────────┘

┌─── COORDINATION METRICS ──────────────────────────────────┐
│                                                            │
│  drone_workload                                            │
│    Number of cells visited per drone. Should be balanced.  │
│                                                            │
│  workload_std                                              │
│    Standard deviation of workload across drones.           │
│    0 = perfectly balanced. High = unfair distribution.     │
│                                                            │
│  unique_contribution                                       │
│    Cells each drone was the FIRST to visit. Shows how      │
│    well drones partition space among themselves.            │
│                                                            │
│  avg_inter_drone_spread_m                                  │
│    Average pairwise distance between drones (meters),      │
│    averaged over all steps. Higher = better separation.    │
│                                                            │
│  avg_heading_diversity                                     │
│    Circular variance of drone headings (0 to 1).           │
│    0 = all drones face same direction (bad for coverage).  │
│    1 = maximally diverse headings (good).                  │
│                                                            │
└────────────────────────────────────────────────────────────┘

┌─── GRADING FORMULA ───────────────────────────────────────┐
│                                                            │
│  Score = 50% × coverage_pct                                │
│        + 10% × efficiency_score (normalized to 100)        │
│        + 10% × (100 - overlap_ratio_pct)                   │
│        + 20% × max(0, 100 - collisions×5)                  │
│        + 10% × maintained_coverage_pct                     │
│                                                            │
│  Grades: A ≥ 90 | B ≥ 80 | C ≥ 70 | D ≥ 60 | F < 60     │
│                                                            │
└────────────────────────────────────────────────────────────┘

┌─── ENVIRONMENT PARAMETERS ────────────────────────────────┐
│                                                            │
│  grid_shape          Grid dimensions (rows × cols)         │
│  grid_resolution_m   Physical size of each cell (meters)   │
│  n_drones            Number of cooperative agents          │
│  drone_safety_radius Min allowed inter-drone distance (m)  │
│  wall_safety_margin  Min distance from boundary (m)        │
│  decay_rate          Per-step decay for maintained cov.    │
│  world_w / world_h   Physical world dimensions (m)         │
│  max_steps           Maximum episode length                │
│                                                            │
└────────────────────────────────────────────────────────────┘
""")
        print("=" * W)

    def plot_visit_heatmap(self, polygon_vertices=None, drone_positions=None,
                          save_path="visit_heatmap.png", show=True):
        """
        Plot a heatmap of visit counts with red intensity gradient.
        Ranks: 0 (black), 1, 2, 3, 4, 5, 6+ (darkest red).
        """
        import matplotlib.pyplot as plt
        import matplotlib.colors as mcolors

        fig, ax = plt.subplots(figsize=(9, 8))
        ext = (-self.world_w / 2, self.world_w / 2,
               -self.world_h / 2, self.world_h / 2)

        # Build display array: -1 = inactive/margin (black), 0 = unseen (green), 1+ = visited (red)
        display = self.visited.copy().astype(np.float32)
        # Mark inactive/margin cells as -1 (black)
        if hasattr(self, '_active_mask') and self._active_mask is not None:
            display[~self._active_mask] = -1

        cmap_colors = [
            (0.0, 0.0, 0.0),    # -1: inactive/margin — black
            (0.8, 0.1, 0.1),    # 0 visits — red (unseen active)
            (0.85, 1.00, 0.85),  # 1 visit  — very light green
            (0.60, 1.00, 0.60),  # 2 visits — light green
            (0.35, 0.85, 0.35),  # 3 visits — medium green
            (0.15, 0.70, 0.15),  # 4 visits — green
            (0.05, 0.55, 0.05),  # 5 visits — dark green
            (0.00, 0.35, 0.00),  # 6+ visits — darkest green
        ]
        bounds = [-1.5, -0.5, 0.5, 1.5, 2.5, 3.5, 4.5, 5.5, 100]
        cmap = mcolors.ListedColormap(cmap_colors)
        norm = mcolors.BoundaryNorm(bounds, cmap.N)

        im = ax.imshow(display, origin='lower', extent=ext,
                        interpolation='nearest', cmap=cmap, norm=norm)

        # Colorbar with labels
        cbar = fig.colorbar(im, ax=ax, ticks=[0.25, 1, 2, 3, 4, 5, 6],
                            shrink=0.85, pad=0.02)
        cbar.ax.set_yticklabels(['0', '1', '2', '3', '4', '5', '6+'])
        cbar.set_label('Visit count', fontsize=11)

        # Polygon outline
        verts = polygon_vertices if polygon_vertices is not None else self.polygon_vertices
        if verts is not None:
            closed = np.vstack([verts, verts[0:1]])
            ax.plot(closed[:, 0], closed[:, 1], 'w-', linewidth=2.0, alpha=0.9)
            ax.plot(closed[:, 0], closed[:, 1], 'k--', linewidth=0.8, alpha=0.5)

        # Drone positions
        if drone_positions is not None:
            colors = plt.get_cmap('tab10')(np.linspace(0, 1, len(drone_positions)))
            for i, pos in enumerate(drone_positions):
                ax.scatter(pos[0], pos[1], s=90, c=[colors[i]],
                           edgecolors='white', linewidths=1.5, zorder=5)
                ax.annotate(f'D{i}', (pos[0] + 0.4, pos[1] + 0.4),
                            fontsize=9, color='white', fontweight='bold', zorder=6)

        # Stats text
        if self._active_mask is not None:
            covered = np.count_nonzero(self.visited[self._active_mask])
        else:
            covered = np.count_nonzero(self.visited)
        v = self.visited[self.visited > 0]
        stats_text = (
            f"Coverage: {covered}/{self.active_cells} ({covered/self.active_cells*100:.1f}%)\n"
            f"1 visit:  {int(np.sum(self.visited == 1))} cells\n"
            f"2 visits: {int(np.sum(self.visited == 2))} cells\n"
            f"3-4:      {int(np.sum((self.visited >= 3) & (self.visited <= 4)))} cells\n"
            f"5+:       {int(np.sum(self.visited >= 5))} cells\n"
            f"Max:      {int(self.visited.max())} visits\n"
            f"Mean:     {v.mean():.1f} (visited only)" if len(v) > 0 else "No cells visited"
        )
        ax.text(0.02, 0.98, stats_text, transform=ax.transAxes, fontsize=9,
                verticalalignment='top', fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='black', alpha=0.7),
                color='white')

        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.set_title('Drone Visit Heatmap', fontsize=14, fontweight='bold')
        ax.set_facecolor('#1a1a1a')
        fig.tight_layout()

        if save_path:
            fig.savefig(save_path, dpi=150, bbox_inches='tight')
            print(f"  Heatmap saved to {save_path}")
        if show:
            plt.show()
        else:
            plt.close(fig)

    # ------------------- ADDITIONAL PLOTTING & DASHBOARD -------------------
    def plot_time_series(self, save_path="time_series.png", show=False):
        """Plot coverage, maintained coverage, new cells per step, spread and heading diversity over time."""
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(4, 1, figsize=(12, 14), sharex=True)

        t = np.arange(1, len(self.coverage_over_time) + 1)
        if len(t) == 0:
            print("  No time-series data to plot.")
            return

        # Panel 1: Coverage curves
        axes[0].plot(t, self.coverage_over_time, label='Cumulative Coverage %', color='#2196F3', linewidth=2)
        axes[0].plot(t, self.maintained_over_time, label='Maintained Coverage %', color='#FF9800', linewidth=1.5, linestyle='--')
        axes[0].axhline(100, color='green', linestyle=':', alpha=0.5, label='100% target')
        for thresh in [50, 75, 90, 95]:
            axes[0].axhline(thresh, color='gray', linestyle=':', alpha=0.2)
        axes[0].set_ylabel('Coverage %')
        axes[0].set_ylim(0, 105)
        axes[0].legend(loc='lower right')
        axes[0].set_title('Coverage Progress\n(Cumulative = ever visited; Maintained = recently refreshed within decay window)')
        axes[0].grid(True, alpha=0.3)

        # Panel 2: Coverage rate (new cells per step) with rolling average
        axes[1].bar(t, self.new_cells_per_step, alpha=0.3, color='#4CAF50', label='New cells / step (raw)')
        window = min(20, len(t) // 5 + 1)
        if window > 1:
            rolling = np.convolve(self.new_cells_per_step, np.ones(window)/window, mode='same')
            axes[1].plot(t, rolling, color='#2E7D32', linewidth=2, label=f'Rolling avg (window={window})')
        axes[1].set_ylabel('New cells / step')
        axes[1].set_title('Exploration Rate\n(How many previously-unseen cells are discovered each step; decline signals saturation)')
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)

        # Panel 3: Inter-drone spread
        axes[2].plot(t, self.spread_over_time, label='Avg pairwise distance (m)', color='#9C27B0', linewidth=1.5)
        axes[2].axhline(np.mean(self.spread_over_time), color='#9C27B0', linestyle='--', alpha=0.5, label='Mean spread')
        axes[2].set_ylabel('Distance [m]')
        axes[2].set_title('Spatial Spread Between Drones\n(Higher = drones are well separated; low = clustering/collision risk)')
        axes[2].legend()
        axes[2].grid(True, alpha=0.3)

        # Panel 4: Heading diversity
        axes[3].plot(t, self.heading_diversity_over_time, label='Heading diversity (circular var)', color='#E91E63', linewidth=1.5)
        axes[3].axhline(np.mean(self.heading_diversity_over_time), color='#E91E63', linestyle='--', alpha=0.5)
        axes[3].set_ylabel('Diversity [0-1]')
        axes[3].set_xlabel('Step')
        axes[3].set_title('Heading Diversity\n(0 = all drones face same direction; 1 = maximally diverse headings — better for coverage)')
        axes[3].legend()
        axes[3].grid(True, alpha=0.3)

        fig.suptitle('Coverage & Behavior Over Time', fontsize=14, fontweight='bold', y=0.995)
        fig.tight_layout(rect=(0, 0, 1, 0.98))
        fig.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"  Time-series saved to {save_path}")
        if show:
            plt.show()
        else:
            plt.close(fig)

    def plot_per_drone_paths(self, save_path="drone_paths.png", show=False):
        """Plot world-space trajectories for each drone with start/end markers and polygon."""
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(9, 8))
        colors = plt.get_cmap('tab10')(np.linspace(0, 1, max(self.n_drones, 3)))
        any_plot = False
        for d in range(self.n_drones):
            wpath = self.drone_world_paths.get(d, [])
            if len(wpath) >= 2:
                pts = np.array(wpath)
                ax.plot(pts[:, 0], pts[:, 1], '-', color=colors[d % len(colors)],
                        label=f'D{d} ({len(pts)} pts)', alpha=0.7, linewidth=1.2)
                ax.scatter(pts[0, 0], pts[0, 1], marker='o', color=colors[d % len(colors)], s=60, zorder=5)
                ax.scatter(pts[-1, 0], pts[-1, 1], marker='X', color=colors[d % len(colors)], s=80, zorder=5)
                any_plot = True
        if not any_plot:
            print("  No world paths available to plot.")
            return
        # Polygon outline
        if self.polygon_vertices is not None:
            verts = np.vstack([self.polygon_vertices, self.polygon_vertices[0:1]])
            ax.plot(verts[:, 0], verts[:, 1], 'k-', linewidth=2, label='Boundary')
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.set_title('Per-drone Trajectories\n(○ = start, ✕ = end)')
        ax.legend(loc='upper right', fontsize=8)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"  Drone paths saved to {save_path}")
        if show:
            plt.show()
        else:
            plt.close(fig)

    def plot_collision_timeline(self, save_path="collision_timeline.png", show=False):
        """Show collisions over time as a scatter plot (step vs pair index)."""
        import matplotlib.pyplot as plt
        if not self.collision_events:
            print("  No collision events to plot.")
            return
        pairs = [f"{e[1]}-{e[2]}" for e in self.collision_events]
        unique_pairs = list(sorted(set(pairs)))
        pair_idx = [unique_pairs.index(p) for p in pairs]
        steps = [e[0] for e in self.collision_events]
        dists = [e[3] for e in self.collision_events]

        fig, ax = plt.subplots(figsize=(10, 4))
        sc = ax.scatter(steps, pair_idx, c=dists, cmap='viridis', s=60, edgecolors='k', linewidths=0.5)
        ax.set_yticks(range(len(unique_pairs)))
        ax.set_yticklabels(unique_pairs)
        ax.set_xlabel('Step')
        ax.set_ylabel('Drone pair')
        ax.set_title('Collision Events Over Time\n(Each dot = one step where two drones were closer than safety radius; color = actual distance)')
        fig.colorbar(sc, ax=ax, label='Distance [m]')
        fig.tight_layout()
        fig.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"  Collision timeline saved to {save_path}")
        if show:
            plt.show()
        else:
            plt.close(fig)

    def plot_workload_distribution(self, save_path="workload_distribution.png", show=False):
        """Bar chart showing per-drone workload: cells visited, unique contribution, and distance."""
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(1, 3, figsize=(15, 5))

        drone_ids = list(range(self.n_drones))
        colors = plt.get_cmap('tab10')(np.linspace(0, 1, max(self.n_drones, 3)))

        # Panel 1: Total cells visited per drone
        cell_counts = [len(self.drone_paths[d]) for d in drone_ids]
        axes[0].bar(drone_ids, cell_counts, color=colors)
        axes[0].axhline(np.mean(cell_counts), color='red', linestyle='--', label=f'Mean={np.mean(cell_counts):.0f}')
        axes[0].set_xlabel('Drone ID')
        axes[0].set_ylabel('Steps taken')
        axes[0].set_title('Steps Per Drone\n(Should be balanced for fair workload)')
        axes[0].legend()

        # Panel 2: Unique cells first discovered
        unique = [int(np.sum(self.first_visitor == d)) for d in drone_ids]
        axes[1].bar(drone_ids, unique, color=colors)
        axes[1].axhline(np.mean(unique), color='red', linestyle='--', label=f'Mean={np.mean(unique):.0f}')
        axes[1].set_xlabel('Drone ID')
        axes[1].set_ylabel('Unique cells discovered')
        axes[1].set_title('Unique Contribution\n(Cells this drone was first to visit)')
        axes[1].legend()

        # Panel 3: Total distance flown
        dists = []
        for d in drone_ids:
            wpath = self.drone_world_paths.get(d, [])
            if len(wpath) >= 2:
                pts = np.array(wpath)
                dists.append(float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1))))
            else:
                dists.append(0.0)
        axes[2].bar(drone_ids, dists, color=colors)
        axes[2].axhline(np.mean(dists), color='red', linestyle='--', label=f'Mean={np.mean(dists):.1f}m')
        axes[2].set_xlabel('Drone ID')
        axes[2].set_ylabel('Distance [m]')
        axes[2].set_title('Distance Flown Per Drone\n(Large disparity = uneven energy usage)')
        axes[2].legend()

        fig.suptitle('Workload Distribution Across Drones', fontsize=13, fontweight='bold')
        fig.tight_layout(rect=(0, 0, 1, 0.94))
        fig.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"  Workload distribution saved to {save_path}")
        if show:
            plt.show()
        else:
            plt.close(fig)

    def plot_coverage_uniformity(self, save_path="coverage_uniformity.png", show=False):
        """Histogram of visit counts across cells to show coverage uniformity."""
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(1, 2, figsize=(12, 5))

        # Only consider active cells
        if self._active_mask is not None:
            visits = self.visited[self._active_mask].flatten()
        else:
            visits = self.visited.flatten()

        # Panel 1: Histogram of visit counts
        max_v = int(visits.max()) if len(visits) > 0 else 1
        bins = np.arange(0, min(max_v + 2, 30))
        axes[0].hist(visits, bins=bins, color='#42A5F5', edgecolor='white', linewidth=0.5)
        axes[0].axvline(np.mean(visits), color='red', linestyle='--', label=f'Mean={np.mean(visits):.2f}')
        axes[0].axvline(np.median(visits), color='orange', linestyle='--', label=f'Median={np.median(visits):.0f}')
        axes[0].set_xlabel('Number of visits')
        axes[0].set_ylabel('Number of cells')
        axes[0].set_title('Visit Count Distribution\n(Ideal: narrow peak at 1-2 visits; fat tail = excessive revisits)')
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)

        # Panel 2: Coverage age map (how stale each cell is)
        age_display = self.coverage_age.copy()
        if self._active_mask is not None:
            age_display[~self._active_mask] = -1
        im = axes[1].imshow(age_display, origin='lower', cmap='hot_r',
                           extent=(-self.world_w/2, self.world_w/2, -self.world_h/2, self.world_h/2))
        axes[1].set_title('Coverage Staleness Map\n(Brighter = more recently visited; dark = stale/needs refresh)')
        axes[1].set_xlabel('x [m]')
        axes[1].set_ylabel('y [m]')
        fig.colorbar(im, ax=axes[1], label='Steps since last visit')

        fig.tight_layout()
        fig.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"  Coverage uniformity saved to {save_path}")
        if show:
            plt.show()
        else:
            plt.close(fig)

    def plot_territory_map(self, save_path="territory_map.png", show=False):
        """Color each cell by which drone first visited it (territory partition).

        Uses high-contrast, maximally-distinct colors with no blending.
        Each cell is colored by its discovering drone only.
        """
        import matplotlib.pyplot as plt
        import matplotlib.colors as mcolors

        fig, ax = plt.subplots(figsize=(9, 8))
        ext = (-self.world_w / 2, self.world_w / 2, -self.world_h / 2, self.world_h / 2)

        display = self.first_visitor.astype(np.float32)
        # -1 = not visited; mark inactive as -2
        if self._active_mask is not None:
            display[~self._active_mask] = -2

        # High-contrast colors: inactive (black), unvisited (dark gray),
        # then maximally distinct per-drone colors (no blending)
        _DRONE_COLORS = [
            (0.12, 0.47, 0.71),  # blue
            (1.00, 0.50, 0.05),  # orange
            (0.17, 0.63, 0.17),  # green
            (0.84, 0.15, 0.16),  # red
            (0.58, 0.40, 0.74),  # purple
            (0.55, 0.34, 0.29),  # brown
            (0.89, 0.47, 0.76),  # pink
            (0.50, 0.50, 0.50),  # gray
            (0.74, 0.74, 0.13),  # olive
            (0.09, 0.75, 0.81),  # cyan
        ]
        cmap_colors = [(0.05, 0.05, 0.05), (0.25, 0.25, 0.25)]  # inactive, unvisited
        for i in range(self.n_drones):
            cmap_colors.append(_DRONE_COLORS[i % len(_DRONE_COLORS)])
        bounds = [-2.5, -1.5, -0.5] + [i + 0.5 for i in range(self.n_drones)]
        cmap = mcolors.ListedColormap(cmap_colors)
        norm = mcolors.BoundaryNorm(bounds, cmap.N)

        im = ax.imshow(display, origin='lower', extent=ext, interpolation='nearest',
                       cmap=cmap, norm=norm)

        # Legend with per-drone cell counts
        from matplotlib.patches import Patch
        unvisited_count = int(np.sum(display == -1))
        legend_elements = [Patch(facecolor=(0.25, 0.25, 0.25),
                                 label=f'Unvisited ({unvisited_count} cells)')]
        for i in range(self.n_drones):
            cnt = int(np.sum(self.first_visitor == i))
            legend_elements.append(
                Patch(facecolor=_DRONE_COLORS[i % len(_DRONE_COLORS)],
                      label=f'Drone {i}  ({cnt} cells)'))
        ax.legend(handles=legend_elements, loc='upper right', fontsize=8,
                  framealpha=0.9, edgecolor='white')

        if self.polygon_vertices is not None:
            verts = np.vstack([self.polygon_vertices, self.polygon_vertices[0:1]])
            ax.plot(verts[:, 0], verts[:, 1], 'w-', linewidth=2)

        # Summary annotation
        total_assigned = int(np.sum(self.first_visitor >= 0))
        ax.text(0.02, 0.02,
                f"Total cells assigned: {total_assigned}\n"
                f"Unvisited: {unvisited_count}",
                transform=ax.transAxes, fontsize=9, verticalalignment='bottom',
                fontfamily='monospace', color='white',
                bbox=dict(boxstyle='round', facecolor='black', alpha=0.7))

        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.set_title('Territory Map — First Discovery Ownership\n'
                     '(Each cell colored by the drone that discovered it FIRST; no blending)')
        ax.set_facecolor('#0a0a0a')
        fig.tight_layout()
        fig.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"  Territory map saved to {save_path}")
        if show:
            plt.show()
        else:
            plt.close(fig)

    def plot_cumulative_distance_vs_coverage(self, save_path="dist_vs_coverage.png", show=False):
        """Plot cumulative distance flown vs coverage achieved (efficiency curve)."""
        import matplotlib.pyplot as plt

        # Compute cumulative distance over time
        cum_dist = [0.0]
        for step_i in range(1, self.step_count):
            step_dist = 0.0
            for d in range(self.n_drones):
                wpath = self.drone_world_paths.get(d, [])
                if len(wpath) > step_i:
                    step_dist += float(np.linalg.norm(
                        np.array(wpath[step_i]) - np.array(wpath[step_i - 1])))
            cum_dist.append(cum_dist[-1] + step_dist)

        if len(cum_dist) < 2:
            print("  Not enough data for distance vs coverage plot.")
            return

        fig, ax = plt.subplots(figsize=(10, 6))
        cov = self.coverage_over_time[:len(cum_dist)]
        ax.plot(cum_dist, cov, color='#2196F3', linewidth=2)
        ax.set_xlabel('Cumulative Distance Flown (all drones) [m]')
        ax.set_ylabel('Coverage %')
        ax.set_title('Distance Efficiency Curve\n(Steeper = more coverage per meter flown; flattening = diminishing returns)')
        ax.grid(True, alpha=0.3)

        # Annotate key thresholds
        for thresh in [50, 75, 90, 95]:
            for i, c in enumerate(cov):
                if c >= thresh:
                    ax.axhline(thresh, color='gray', linestyle=':', alpha=0.3)
                    ax.annotate(f'{thresh}% @ {cum_dist[i]:.0f}m',
                               xy=(cum_dist[i], thresh), fontsize=8, color='gray')
                    break

        fig.tight_layout()
        fig.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"  Distance vs coverage saved to {save_path}")
        if show:
            plt.show()
        else:
            plt.close(fig)

    def plot_speed_profile(self, save_path="speed_profile.png", show=False):
        """Plot instantaneous speed of each drone over time."""
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(10, 5))
        colors = plt.get_cmap('tab10')(np.linspace(0, 1, max(self.n_drones, 3)))
        any_plot = False
        for d in range(self.n_drones):
            wpath = self.drone_world_paths.get(d, [])
            if len(wpath) >= 2:
                pts = np.array(wpath)
                speeds = np.linalg.norm(np.diff(pts, axis=0), axis=1)
                ax.plot(np.arange(1, len(speeds) + 1), speeds, color=colors[d % len(colors)],
                        alpha=0.6, linewidth=1, label=f'D{d}')
                any_plot = True
        if not any_plot:
            print("  No speed data to plot.")
            return
        ax.set_xlabel('Step')
        ax.set_ylabel('Speed [m/step]')
        ax.set_title('Drone Speed Profile Over Time\n(Drops to zero = stagnation; spikes = aggressive maneuvers)')
        ax.legend(loc='upper right', fontsize=8)
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        fig.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"  Speed profile saved to {save_path}")
        if show:
            plt.show()
        else:
            plt.close(fig)

    def _explain_parameters_html(self):
        """Return an HTML block explaining all parameters and metrics in detail."""
        sections = {
            'Environment Parameters': {
                'grid_shape': 'Grid dimensions (rows × cols). The environment is discretized into this many cells. Finer grids give more precision but require more steps to cover.',
                'grid_resolution_m': 'Physical size of each grid cell in meters. Combined with grid_shape, determines the total world size.',
                'n_drones': 'Number of cooperative agents (drones) participating in the coverage task simultaneously.',
                'drone_safety_radius': 'Minimum allowed distance (m) between any two drones. If closer, a collision event is logged. Typical real-world value: 1.5-3m.',
                'wall_safety_margin': 'Minimum distance (m) a drone should maintain from the polygon boundary. Violations indicate boundary-unsafe behavior.',
                'decay_rate': 'Per-step multiplicative decay (0..1) for "maintained" coverage. A cell visited T steps ago has weight decay_rate^T. Higher values mean cells stay "fresh" longer. Window ≈ 1/(1-decay_rate) steps.',
                'world_w / world_h': 'Physical dimensions of the world in meters.',
                'max_steps': 'Maximum episode length. The simulation terminates after this many steps regardless of coverage achieved.',
            },
            'Coverage Metrics': {
                'coverage_pct': 'Percentage of active-area cells visited at least once. The primary success metric. 100% = full coverage.',
                'maintained_coverage_pct': 'Percentage of cells that have been visited recently enough to still be "fresh" (within the decay window). Important for persistent surveillance tasks.',
                'cells_covered': 'Absolute number of unique cells visited at least once.',
                'total_cells': 'Number of active cells inside the polygon (excluding margin/inactive areas).',
                'steps_to_threshold': 'Number of steps required to reach each coverage milestone (50%, 75%, 90%, 95%, 98%, 100%). "NOT REACHED" means the threshold was never achieved.',
            },
            'Efficiency Metrics': {
                'efficiency_score': 'cells_covered / (total_steps × n_drones). Represents how productively each drone-step was used. Ideal ≈ 1.0 (every step discovers a new cell). Typical good: 0.3-0.6.',
                'overlap_ratio_pct': 'Percentage of total cell-visits that were redundant (cell already visited). Lower = less wasted effort. 0% = perfect (no revisits).',
                'distance_per_new_cell_m': 'Average distance (m) flown to discover each new cell. Lower = more efficient paths.',
                'coverage_rate_slowdown': 'Ratio of (early_rate - late_rate) / early_rate. 0 = constant rate; 1 = exploration completely stopped. Shows how much the policy struggles in late-game.',
            },
            'Safety Metrics': {
                'collision_count': 'Total number of step×pair instances where two drones were closer than drone_safety_radius. Each occurrence is counted separately.',
                'wall_violation_count': 'Total number of step×drone instances where a drone was outside the polygon boundary.',
            },
            'Path Quality Metrics': {
                'total_distance_m': 'Sum of all distances flown by all drones (meters).',
                'drone_distances_m': 'Distance flown by each individual drone.',
                'energy_estimate': 'Approximate energy usage = total_distance + 0.5 × total_turning_radians. Penalizes excessive turning which costs battery.',
                'circling_events': 'Number of 40-step windows where a drone visited fewer than 8 unique cells (indicating it was circling/stuck).',
                'stagnation_events': 'Per-drone count of 20-step windows where the drone visited ≤2 unique cells (completely stuck).',
            },
            'Coordination Metrics': {
                'drone_workload': 'Number of steps each drone was active / moved. Should be balanced.',
                'workload_std': 'Standard deviation of workload across drones. Lower = fairer distribution. 0 = perfectly balanced.',
                'unique_contribution': 'Number of cells each drone was the FIRST to visit. Shows how well drones partition the space.',
                'avg_inter_drone_spread_m': 'Average pairwise distance between drones (meters), averaged over all steps. Higher = better spatial distribution.',
                'avg_heading_diversity': 'Circular variance of drone headings (0-1). 0 = all flying same direction (bad for coverage); 1 = maximally diverse headings (good).',
            },
            'Grading': {
                'Overall Grade': 'Weighted score: 50% coverage + 10% efficiency + 10% (100-overlap) + 20% safety (penalized by collisions) + 10% maintained coverage. Grades: A≥90, B≥80, C≥70, D≥60, F<60.',
            },
        }
        html = []
        for section_name, items in sections.items():
            html.append(f"<h3>{section_name}</h3>")
            html.append('<table border="1" cellpadding="6" style="border-collapse:collapse; margin-bottom:16px;">')
            html.append('<tr style="background:#f0f0f0;"><th>Parameter</th><th>Description</th></tr>')
            for k, v in items.items():
                html.append(f'<tr><td><code>{k}</code></td><td>{v}</td></tr>')
            html.append('</table>')
        return '\n'.join(html)

    def generate_dashboard(self, out_dir="outputs/eval", prefix="validation"):
        """Create a comprehensive dashboard (HTML + PNGs) summarising results and explaining metrics.

        Saves files into out_dir and prints the index location.
        """
        out_dir = Path(out_dir)
        out_dir.mkdir(parents=True, exist_ok=True)

        # Define all plots to generate
        plot_specs = [
            ('visit_heatmap', lambda p: self.plot_visit_heatmap(save_path=p, show=False)),
            ('time_series', lambda p: self.plot_time_series(save_path=p, show=False)),
            ('drone_paths', lambda p: self.plot_per_drone_paths(save_path=p, show=False)),
            ('collision_timeline', lambda p: self.plot_collision_timeline(save_path=p, show=False)),
            ('workload_distribution', lambda p: self.plot_workload_distribution(save_path=p, show=False)),
            ('coverage_uniformity', lambda p: self.plot_coverage_uniformity(save_path=p, show=False)),
            ('territory_map', lambda p: self.plot_territory_map(save_path=p, show=False)),
            ('dist_vs_coverage', lambda p: self.plot_cumulative_distance_vs_coverage(save_path=p, show=False)),
            ('speed_profile', lambda p: self.plot_speed_profile(save_path=p, show=False)),
        ]

        plot_paths = {}
        for name, plot_fn in plot_specs:
            p = str(out_dir / f"{prefix}_{name}.png")
            try:
                plot_fn(p)
                plot_paths[name] = Path(p).name
            except Exception as e:
                print(f"  Warning: failed to produce {name}: {e}")

        # Metrics and parameters
        m = self.get_metrics()

        # Save metrics + basic parameters as JSON for programmatic consumption
        try:
            meta = {
                'metrics': m,
                'params': {
                    'grid_shape': self.grid_shape,
                    'grid_resolution_m': self.grid_res,
                    'n_drones': self.n_drones,
                    'drone_safety_radius': self.drone_safety_radius,
                    'wall_safety_margin': self.wall_safety_margin,
                    'decay_rate': self.decay_rate,
                    'world_w': self.world_w,
                    'world_h': self.world_h,
                    'max_steps': self.max_steps,
                }
            }
            json_p = out_dir / f"{prefix}_metrics.json"
            with open(json_p, 'w', encoding='utf-8') as jf:
                json.dump(meta, jf, indent=2)
            print(f"  Metrics JSON saved to {json_p}")
        except Exception:
            pass

        # Build HTML dashboard
        grade = self._grade(m)
        html = ["""<html><head><meta charset='utf-8'>
<title>Validation Dashboard</title>
<style>
body { font-family: 'Segoe UI', Arial, sans-serif; margin: 20px; background: #fafafa; }
h1 { color: #1a237e; border-bottom: 3px solid #1a237e; padding-bottom: 8px; }
h2 { color: #283593; margin-top: 30px; }
h3 { color: #3949ab; }
table { border-collapse: collapse; margin: 10px 0; }
th, td { border: 1px solid #ccc; padding: 8px 12px; text-align: left; }
th { background: #e8eaf6; }
tr:nth-child(even) { background: #f5f5f5; }
.grade-box { font-size: 28px; font-weight: bold; padding: 16px 32px; border-radius: 8px;
             display: inline-block; margin: 10px 0; }
.grade-A { background: #c8e6c9; color: #1b5e20; }
.grade-B { background: #dcedc8; color: #33691e; }
.grade-C { background: #fff9c4; color: #f57f17; }
.grade-D { background: #ffe0b2; color: #e65100; }
.grade-F { background: #ffcdd2; color: #b71c1c; }
img { max-width: 100%; border: 1px solid #ddd; border-radius: 4px; margin: 8px 0; }
.metric-section { background: white; padding: 16px; border-radius: 8px; margin: 12px 0;
                  box-shadow: 0 1px 3px rgba(0,0,0,0.1); }
code { background: #e8eaf6; padding: 2px 6px; border-radius: 3px; }
.summary-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(280px, 1fr)); gap: 12px; }
.summary-card { background: white; padding: 14px; border-radius: 8px; border-left: 4px solid #3f51b5;
                box-shadow: 0 1px 3px rgba(0,0,0,0.08); }
.summary-card h4 { margin: 0 0 6px 0; color: #3f51b5; }
.summary-card .value { font-size: 22px; font-weight: bold; }
</style></head><body>"""]

        html.append(f"<h1>🛸 Multi-Drone Coverage Validation Dashboard</h1>")
        html.append(f"<p>Generated: {time.strftime('%Y-%m-%d %H:%M:%S')}</p>")

        # Grade
        grade_letter = grade.split('(')[0].strip()
        html.append(f'<div class="grade-box grade-{grade_letter}">Overall Grade: {grade}</div>')

        # Summary cards
        html.append('<h2>📊 Key Metrics at a Glance</h2>')
        html.append('<div class="summary-grid">')
        cards = [
            ('Coverage', f"{m['coverage_pct']}%", 'Cells visited at least once'),
            ('Maintained', f"{m['maintained_coverage_pct']}%", 'Cells still fresh (within decay window)'),
            ('Efficiency', f"{m['efficiency_score']}", 'Cells per drone-step'),
            ('Overlap', f"{m['overlap_ratio_pct']}%", 'Redundant revisits'),
            ('Collisions', f"{m['collision_count']}", 'Safety violations'),
            ('Wall Violations', f"{m['wall_violation_count']}", 'Boundary breaches'),
            ('Total Distance', f"{m['total_distance_m']}m", 'All drones combined'),
            ('Steps', f"{m['total_steps']}", f"of {self.max_steps or '∞'} max"),
        ]
        for title, value, desc in cards:
            html.append(f'<div class="summary-card"><h4>{title}</h4><div class="value">{value}</div><small>{desc}</small></div>')
        html.append('</div>')

        # Detailed metrics table
        html.append('<h2>📋 Full Metrics Table</h2>')
        html.append('<div class="metric-section">')
        html.append('<table><tr><th>Metric</th><th>Value</th></tr>')
        for k, v in m.items():
            html.append(f"<tr><td><code>{k}</code></td><td>{v}</td></tr>")
        html.append('</table></div>')

        # Coverage thresholds
        html.append('<h2>🎯 Coverage Milestones</h2>')
        html.append('<div class="metric-section">')
        html.append('<table><tr><th>Threshold</th><th>Steps Required</th><th>Status</th></tr>')
        for t, s in m['steps_to_threshold'].items():
            status = f"✅ Reached at step {s}" if s else "❌ Not reached"
            html.append(f"<tr><td>{t}%</td><td>{s if s else '—'}</td><td>{status}</td></tr>")
        html.append('</table></div>')

        # Per-drone breakdown
        html.append('<h2>🤖 Per-Drone Breakdown</h2>')
        html.append('<div class="metric-section">')
        html.append('<table><tr><th>Drone</th><th>Steps</th><th>Unique Cells</th><th>Distance (m)</th></tr>')
        for d in range(self.n_drones):
            wl = m['drone_workload'][d] if d < len(m['drone_workload']) else 0
            uc = m['unique_contribution'][d] if d < len(m['unique_contribution']) else 0
            dd = m['drone_distances_m'][d] if d < len(m['drone_distances_m']) else 0
            html.append(f"<tr><td>Drone {d}</td><td>{wl}</td><td>{uc}</td><td>{dd}</td></tr>")
        html.append('</table></div>')

        # Visualizations
        plot_titles = {
            'visit_heatmap': ('🗺️ Visit Heatmap', 'Shows how many times each cell was visited. Green = visited (darker = more visits). Red = never visited. Black = outside boundary.'),
            'time_series': ('📈 Coverage & Behavior Over Time', '4-panel chart: (1) Coverage % progress, (2) Exploration rate (new cells/step), (3) Spatial spread between drones, (4) Heading diversity.'),
            'drone_paths': ('🛤️ Per-Drone Trajectories', 'World-coordinate paths of each drone. Circle = start position, X = final position. Good policies show non-overlapping sweeps.'),
            'collision_timeline': ('⚠️ Collision Timeline', 'Each dot represents a timestep where two drones were closer than the safety radius. Color indicates actual distance.'),
            'workload_distribution': ('⚖️ Workload Distribution', '3 bar charts: steps taken, unique cells discovered, and distance flown per drone. Red dashed line = mean. Large disparity = unfair workload.'),
            'coverage_uniformity': ('📊 Coverage Uniformity', 'Left: histogram of visit counts (ideal: narrow peak at 1-2). Right: staleness map showing how recently each cell was refreshed.'),
            'territory_map': ('🏁 Territory Map', 'Colors cells by which drone discovered them first. Shows how well the swarm partitions space. Even coloring = good coordination.'),
            'dist_vs_coverage': ('⚡ Distance vs Coverage (Efficiency Curve)', 'Coverage achieved vs total distance flown. Steeper slope = more efficient. Flattening = diminishing returns (harder to find remaining cells).'),
            'speed_profile': ('🏎️ Speed Profile', 'Instantaneous speed of each drone over time. Drops to zero indicate stagnation. Consistent speed = smooth paths.'),
        }

        html.append('<h2>📸 Visualizations</h2>')
        for name, fname in plot_paths.items():
            title, desc = plot_titles.get(name, (name, ''))
            html.append(f'<div class="metric-section">')
            html.append(f'<h3>{title}</h3>')
            html.append(f'<p><em>{desc}</em></p>')
            html.append(f"<img src='{fname}' width='900'/>")
            html.append('</div>')

        # Parameter explanations
        html.append('<h2>📖 Parameter & Metric Reference</h2>')
        html.append('<div class="metric-section">')
        html.append(self._explain_parameters_html())
        html.append('</div>')

        # Interpretation guide
        html.append('<h2>💡 How to Interpret Results</h2>')
        html.append('<div class="metric-section">')
        html.append("""<h3>What makes a good coverage policy?</h3>
<ol>
<li><b>High coverage (>95%)</b> — The primary objective. The swarm should visit every reachable cell.</li>
<li><b>Low overlap (<20%)</b> — Efficient policies don't waste time revisiting cells unnecessarily.</li>
<li><b>Zero collisions</b> — Safety is paramount; drones must maintain safe separation.</li>
<li><b>Balanced workload</b> — All drones should contribute equally (low workload_std).</li>
<li><b>High maintained coverage</b> — For surveillance: cells should be refreshed before they go stale.</li>
<li><b>Fast convergence</b> — Reaching 90%+ coverage quickly (fewer steps) indicates efficient exploration.</li>
<li><b>Good spread</b> — Drones should spread out (high avg spread) to avoid redundant coverage.</li>
<li><b>Diverse headings</b> — Drones facing different directions explore different frontiers simultaneously.</li>
</ol>

<h3>Common Failure Modes</h3>
<ul>
<li><b>Circling/Stagnation</b> — Drone gets stuck in a loop. Check speed profile for zero-speed periods.</li>
<li><b>Clustering</b> — Drones bunch together. Check spread plot for dips and territory map for uneven partition.</li>
<li><b>Boundary hugging</b> — Drone follows walls instead of exploring interior. Check trajectory plot.</li>
<li><b>Late-game inefficiency</b> — Coverage slows dramatically near 90%+. Check rate slowdown metric and distance-vs-coverage curve.</li>
</ul>
</div>""")

        html.append('<h2>📝 Notes</h2>')
        html.append('<div class="metric-section">')
        html.append('<p>This dashboard was auto-generated by <code>CoverageValidator.generate_dashboard()</code>. '
                    'All plots are saved as individual PNGs alongside this HTML file. '
                    'Metrics are also available in JSON format for programmatic analysis.</p>')
        html.append('</div>')
        html.append('</body></html>')

        index_p = out_dir / f"{prefix}_index.html"

        with open(index_p, 'w', encoding='utf-8') as f:
            f.write('\n'.join(html))

        print(f"  Dashboard saved to {index_p}")

# ─────────────────────────────────────────────────────────────
#  Standalone runner
# ─────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import argparse
    import torch
    from env_swarm_polygon import SwarmSearchPolygonEnv
    from model import ActorCritic
    from train_fast import make_env_cfg
    from evaluate_and_visualize import eval_vis

    parser = argparse.ArgumentParser(description="Evaluate swarm coverage model")
    parser.add_argument('--model', type=str, default=None, help='Path to a specific model .pt file to evaluate')
    args = parser.parse_args()

    ROOT = Path(__file__).resolve().parent

    def find_model(custom_path=None) -> str:
        """Load custom model if provided, else find model_100coverage.pt then best_model.pt."""
        if custom_path:
            p = Path(custom_path)
            if not p.exists():
                raise FileNotFoundError(f"Provided model path does not exist: {custom_path}")
            print(f"  Loading custom model: {p}")
            return str(p)

        candidates = ['model_100coverage.pt', 'best_model.pt', 'best_so_far.pt',
                      'latest_checkpoint.pt', 'last_model.pt']
        for name in candidates:
            p = ROOT / 'outputs' / 'models' / name
            if p.exists():
                print(f"  Loading model: {p}")
                return str(p)
        raise FileNotFoundError('No saved model found in outputs/models')

    cfg = make_env_cfg()
    env = SwarmSearchPolygonEnv(cfg)
    device = 'cuda' if torch.cuda.is_available() else 'cpu'

    # Load model
    model_path = find_model(args.model)
    model = ActorCritic(env.actor_obs_spec, env.critic_obs_spec).to(device)
    ckpt = torch.load(model_path, map_location=device)
    # Robust checkpoint loading sequence:
    sd = ckpt.get('model', ckpt)
    try:
        model.load_state_dict(sd)
        print(f"  Model loaded exactly from checkpoint: {model_path}")
    except Exception as e_exact:
        print(f"  Exact state_dict load failed: {e_exact}")
        # Try a non-strict load (will skip missing/unexpected keys)
        try:
            res = model.load_state_dict(sd, strict=False)
            print(f"  Partial load (strict=False) result: {res}")
        except Exception as e_partial:
            print(f"  Partial non-strict load also failed: {e_partial}")

        # Final attempt: copy matching tensors by name and shape
        own_state = model.state_dict()
        copied = 0
        for k, v in sd.items():
            if k in own_state and own_state[k].shape == v.shape:
                own_state[k] = v
                copied += 1
        try:
            model.load_state_dict(own_state)
            # Report actor vs critic loading status
            actor_keys = [k for k in sd if k.startswith('actor.')]
            actor_loaded = sum(1 for k in actor_keys if k in own_state and own_state[k].shape == sd[k].shape)
            critic_keys = [k for k in sd if k.startswith('critic.')]
            critic_loaded = sum(1 for k in critic_keys if k in own_state and own_state[k].shape == sd[k].shape)
            print(f"  Copied {copied} tensors from checkpoint that matched shape and loaded them.")
            print(f"    Actor:  {actor_loaded}/{len(actor_keys)} params loaded (used for inference)")
            print(f"    Critic: {critic_loaded}/{len(critic_keys)} params loaded (NOT used for inference)")
            if actor_loaded == len(actor_keys):
                print("  ✅ All actor weights loaded successfully — inference will be correct.")
            else:
                print(f"  ⚠️  {len(actor_keys) - actor_loaded} actor params could not be loaded — results may be unreliable!")
        except Exception as e_final:
            print(f"  Final shaped-copy load failed: {e_final}")
            print("  At this point the checkpoint and model architecture are incompatible.")
            raise
    model.eval()

    # ── Run validation on MULTIPLE polygon seeds ──
    # A single seed can be misleading — the model may perform well on some
    # polygon shapes and poorly on others.  We run 5 seeds and report all.
    # Each seed gets its own subfolder: outputs/eval/seed_XXX/
    # Generate 10 truly random seeds for every validation run
    # Using 0 to 10000 range for seeds
    EVAL_SEEDS = np.random.randint(0, 10000, size=20).tolist()
    all_metrics = []
    all_coverages = []
    all_validators = []
    eval_root = ROOT / 'outputs' / 'eval'
    os.makedirs(eval_root, exist_ok=True)

    # ── Clean up existing seed_* folders before saving new results ──
    for entry in eval_root.iterdir():
        if entry.is_dir() and entry.name.startswith('seed_'):
            shutil.rmtree(entry)
            print(f"  Removed old eval folder: {entry.name}/")

    for seed_idx, seed in enumerate(EVAL_SEEDS):
        print(f"\n{'='*70}")
        print(f"  VALIDATION RUN {seed_idx+1}/{len(EVAL_SEEDS)}  (seed={seed})")
        print(f"{'='*70}")

        obs, info = env.reset(seed=seed)

        validator = CoverageValidator(
            grid_shape=(env.grid_h, env.grid_w),
            n_drones=cfg.max_agents,
            grid_resolution=cfg.grid_resolution_m,
            drone_safety_radius=cfg.drone_safety_radius,
            wall_safety_margin=cfg.wall_safety_margin,
            decay_rate=cfg.trajectory_heatmap_decay,
            polygon_vertices=env.polygon_vertices,
            world_w=cfg.world_w,
            world_h=cfg.world_h,
            max_steps=env.max_steps,
        )
        validator._active_mask = env.active_area_mask.copy()

        max_steps = env.max_steps
        for step in range(max_steps):
            obs_t = {k: torch.as_tensor(v, dtype=torch.float32, device=device) for k, v in obs.items()}
            with torch.no_grad():
                actions_t, _ = model.actor.act(
                    obs_t['maps'], obs_t['self_state'],
                    obs_t['neighbor_state'], obs_t['neighbor_mask'],
                    deterministic=True)
            actions = actions_t.cpu().numpy()

            obs, reward, terminated, truncated, info = env.step(actions)
            done = terminated or truncated

            grid_positions = []
            for i in range(cfg.max_agents):
                gx = int((env.pos[i, 0] + cfg.world_w / 2) / cfg.grid_resolution_m)
                gy = int((env.pos[i, 1] + cfg.world_h / 2) / cfg.grid_resolution_m)
                gx = np.clip(gx, 0, env.grid_w - 1)
                gy = np.clip(gy, 0, env.grid_h - 1)
                grid_positions.append((gy, gx))

            validator.record_step(
                drone_positions=grid_positions,
                drone_world_positions=env.pos[:cfg.max_agents].tolist(),
                drone_headings=env.heading[:cfg.max_agents].tolist(),
            )

            validator.sync_coverage_from_fov(env.ever_visited, grid_positions)

            if done:
                break

        validator.print_report()
        metrics = validator.get_metrics()
        all_metrics.append(metrics)
        all_coverages.append(metrics['coverage_pct'])
        all_validators.append(validator)

        # ── Per-seed output folder ──
        seed_dir = str(eval_root / f"seed_{seed}")
        os.makedirs(seed_dir, exist_ok=True)

        # Generate full dashboard into seed folder
        validator.generate_dashboard(out_dir=seed_dir, prefix="validation")

        # Save per-seed heatmap
        validator.plot_visit_heatmap(
            polygon_vertices=env.polygon_vertices,
            drone_positions=env.pos[:cfg.max_agents].tolist(),
            save_path=str(Path(seed_dir) / 'visit_heatmap.png'),
            show=False,
        )

        # Save per-seed results.txt
        seed_results_path = Path(seed_dir) / 'results.txt'
        with open(seed_results_path, 'w', encoding='utf-8') as f:
            f.write(f"Seed: {seed}\n")
            f.write(f"Model: {model_path}\n")
            f.write("=" * 60 + "\n")
            for key, value in metrics.items():
                f.write(f"  {key}: {value}\n")
            f.write("=" * 60 + "\n")
            f.write(f"GRADE: {validator._grade(metrics)}\n")
        print(f"  Seed {seed} results saved to {seed_dir}/")

    # ── Aggregate report ──
    print(f"\n{'='*70}")
    print(f"  AGGREGATE RESULTS ACROSS {len(EVAL_SEEDS)} POLYGON SEEDS")
    print(f"{'='*70}")
    print(f"  Seeds:        {EVAL_SEEDS}")
    print(f"  Coverages:    {[f'{c:.1f}%' for c in all_coverages]}")
    print(f"  Mean:         {np.mean(all_coverages):.2f}%")
    print(f"  Min:          {np.min(all_coverages):.2f}%")
    print(f"  Max:          {np.max(all_coverages):.2f}%")
    print(f"  Std:          {np.std(all_coverages):.2f}%")
    collisions = [m['collision_count'] for m in all_metrics]
    wall_collisions = [m['wall_collision_count'] for m in all_metrics]
    wall_warnings = [m['wall_violation_count'] for m in all_metrics]
    print(f"  Drone collisions: {collisions}  (total={sum(collisions)})")
    print(f"  Wall collisions:  {wall_collisions}  (total={sum(wall_collisions)})  ← HARD FAIL")
    print(f"  Wall proximity:   {wall_warnings}  (total={sum(wall_warnings)})  ← warning only")
    if np.min(all_coverages) >= 99.0 and sum(collisions) == 0 and sum(wall_collisions) == 0:
        print(f"\n  ✅ MODEL PASSES: ≥99% on ALL seeds, zero collisions")
    else:
        if np.min(all_coverages) < 99.0:
            print(f"\n  ❌ COVERAGE FAIL: min coverage {np.min(all_coverages):.1f}% < 99%")
        if sum(collisions) > 0:
            print(f"  ❌ SAFETY FAIL: {sum(collisions)} drone collisions")
        if sum(wall_collisions) > 0:
            print(f"  ❌ SAFETY FAIL: {sum(wall_collisions)} wall collisions (outside polygon)")
    print(f"{'='*70}")

    # ── Save aggregate results to eval root ──
    results_path = eval_root / 'validation_results.txt'
    with open(results_path, 'w', encoding='utf-8') as f:
        f.write("=" * 60 + "\n")
        f.write("         COVERAGE VALIDATION RESULTS\n")
        f.write("=" * 60 + "\n")
        f.write(f"Model: {model_path}\n")
        f.write(f"Seeds: {EVAL_SEEDS}\n\n")
        for i, (seed, metrics) in enumerate(zip(EVAL_SEEDS, all_metrics)):
            f.write(f"--- Seed {seed} (see seed_{seed}/ for full dashboard) ---\n")
            f.write(f"  coverage_pct: {metrics['coverage_pct']}\n")
            f.write(f"  collision_count: {metrics['collision_count']}\n")
            f.write(f"  wall_collision_count: {metrics['wall_collision_count']}\n")
            f.write(f"  wall_violation_count: {metrics['wall_violation_count']}\n")
            f.write(f"  maintained_coverage_pct: {metrics['maintained_coverage_pct']}\n")
            f.write(f"  GRADE: {all_validators[i]._grade(metrics)}\n\n")
        f.write("=" * 60 + "\n")
        f.write(f"AGGREGATE: mean={np.mean(all_coverages):.2f}% "
                f"min={np.min(all_coverages):.2f}% "
                f"max={np.max(all_coverages):.2f}%\n")
        f.write(f"Drone collisions: {sum(collisions)}  Wall collisions: {sum(wall_collisions)}\n")
        if np.min(all_coverages) >= 99.0 and sum(collisions) == 0 and sum(wall_collisions) == 0:
            f.write("VERDICT: PASS\n")
        else:
            f.write("VERDICT: FAIL\n")
    print(f"\n  Aggregate results saved to {results_path}")
    print(f"  Per-seed folders: {[f'seed_{s}/' for s in EVAL_SEEDS]}")

    # eval_vis()
