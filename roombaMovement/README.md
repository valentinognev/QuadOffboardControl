# Roomba-style drone ground scan (pygame)

Python simulation of a drone at fixed altitude scanning a flat **100×100 m** patch with a downward-tilted camera. The ground is discretized into **200×200** cells (**0.5×0.5 m**). **Pygame** shows a live heatmap; motion and yaw come from time-series **velocity/yaw-rate** histories (or built-in trajectories).

---

## What it does

1. **Kinematics** — World-frame `vx`, `vy` (m/s) and yaw rate `ω` (rad/s) are sampled from a `MotionHistory` via `numpy.interp` over time `t`.
2. **Camera** — Pinhole-style frustum with horizontal/vertical FOV; optical axis tilted **α** from nadir; footprint on `z = 0` is a convex quadrilateral (with ordering and grazing-ray handling for stability).
3. **Coverage / color** — Each cell stores a scalar **color value in [0, 1]** where **0 = red** and **1 = blue** (logical scale). **RGB on screen** is a linear blend of fixed red/blue endpoint colors from that scalar only.
4. **Accumulation rules** — While a cell’s center lies **inside the camera footprint on the ground**, **and** within a configurable **horizontal distance** of the drone, its value increases linearly: `+ VIEWSLOPE × dt`, capped at 1. All other cells are multiplied each step by `exp(-dt / τ)` (off-view exponential decay).
5. **Snake trajectory (default)** — Boustrophedon (lawn-mower) path inside a margin, then a straight return to the start; **yaw magnitude** is configurable and **sign alternates every horizontal leg** (vertical segments use the next leg’s sign). The history has a finite **`period`**; simulation **resets position and yaw** to the initial pose when each period completes, so the path repeats. **Coverage** (`visited`) still uses the full footprint (no distance limit).

---

## Dependencies

- Python 3.10+ (uses `float | None` style type hints; `from __future__ import annotations` elsewhere)
- **numpy**
- **pygame** (≥2.x)

Install (example):

```bash
pip install numpy pygame
```

---

## How to run

From anywhere (the script prepends its own directory to `sys.path`):

```bash
python /path/to/roombaMovement/main.py
```

Or from the repo:

```bash
cd /path/to/general_infrastructure/roombaMovement
python main.py
```

Optional headless smoke test:

```bash
SDL_VIDEODRIVER=dummy timeout 2 python main.py
```

---

## Project layout

| File | Role |
|------|------|
| `main.py` | Pygame window, HUD, grid rendering (`cell_color_01` → RGB), drone marker and footprint overlay |
| `config.py` | Physical constants, trajectory/color tuning, env-based overrides |
| `simulation.py` | `DroneSim`: integration, cyclic reset, `mark_footprint`, `mean_color_value_01()` |
| `geometry.py` | Frustum ↔ ground plane, footprint mask, distance-limited accumulation mask |
| `motion_history.py` | `MotionHistory`, `load_history`, snake / constant / CSV / JSON |

---

## Coordinates and camera

- **World**: Ground patch `x, y ∈ [0, 100]` m, `z = 0`; drone at `(px, py, ALT)`.
- **Yaw**: About **+Z** (heading in the horizontal plane).
- **Tilt α**: Angle between optical axis and **nadir (−Z)** in the body-forward plane, then rotated by yaw into world.
- **Visited** (coverage fraction): Any cell whose center lies in the footprint polygon and inside the patch is marked `True` for the whole pass (independent of accumulation distance).

---

## Motion history

- **Columns / arrays**: `t`, `vx`, `vy`, `yaw_rate` (strictly increasing `t`; linear interpolation in time).
- **Snake** (`TRAJECTORY=snake`, default): cyclic boustrophedon + return; `MotionHistory.period` set so commands repeat; pose reset each cycle.
- **Constant** (`TRAJECTORY=constant`): `vx = 3`, `vy = 0`, constant signed yaw from config.
- **CSV**: `ROOMBA_HISTORY_CSV=/path/to/file.csv` with header `t,vx,vy,yaw_rate` (overrides trajectory mode). Initial position uses `INITIAL_PX` / `INITIAL_PY` when CSV is set.
- **JSON**: Same keys; optional `period` for cyclic CSV-style data.

---

## Cell color model (summary)

| Concept | Meaning |
|---------|--------|
| `cell_color_01[i,j]` | Per-cell value in **[0, 1]**; **0 → red**, **1 → blue** (logical) |
| In footprint + within `COLOR_ACCUM_MAX_DIST_M` of drone (horizontal) | `value ← min(1, value + VIEWSLOPE * dt)` |
| Otherwise | `value ← value * exp(-dt / DEGRADE_TAU_S)` |
| Display | `RGB = value * blue_rgb + (1 - value) * red_rgb` (see `main.py`) |
| HUD **mean_color** | **Mean of `cell_color_01`** over the grid (same 0–1 scale) |

---

## Environment variables (reference)

| Variable | Effect |
|----------|--------|
| `ROOMBA_TRAJECTORY` | `snake` (default) or `constant` when no CSV |
| `ROOMBA_HISTORY_CSV` | Path to CSV history (overrides trajectory) |
| `ROOMBA_YAW_RATE` | Snake: \|ω\| magnitude (rad/s) |
| `ROOMBA_YAW_SIGN` | Snake: ±1 for first horizontal leg |
| `ROOMBA_CONSTANT_YAW` | Constant mode: signed ω (rad/s) |
| `ROOMBA_VIEWSLOPE` | Rate (1/s) of increase toward 1 while accumulating |
| `ROOMBA_DEGRADE_TAU` | Off-view decay time constant τ (s) |
| `ROOMBA_ACCUM_MAX_DIST` | Max horizontal distance (m) for accumulation; `inf` / `none` / empty = no limit |

Defaults for numeric knobs are in `config.py` (they may differ from this table if you change the file).

---

## IDE / debugging

- **VS Code**: A launch configuration can set `cwd` to `roombaMovement` and `program` to `main.py`; the `sys.path` bootstrap in `main.py` also helps when the debugger’s working directory is the workspace root.

---

## Original intent (brief)

The tool visualizes **drone motion** over a **100×100 m** area with a **scanning camera** (FOV, tilt from nadir), **grid cells** that change color according to **viewing and a 0–1 color value**, optional **time-series** or **snake** paths, **cyclic** repetition for the snake, and a **pygame** UI with **footprint**, **drone position**, and **telemetry** (time, pose, velocities, coverage, slope, τ, accumulation radius, mean color).
