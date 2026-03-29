"""Constants for drone scan simulation."""

from __future__ import annotations

import math
import os

# World patch (meters)
PATCH_SIZE_M = 100.0
CELL_SIZE_M = 0.5
GRID_N = int(PATCH_SIZE_M / CELL_SIZE_M)  # 200

# Drone / camera
ALT_M = 20.0
FOV_DEG_X = 60.0
FOV_DEG_Y = 60.0
# Tilt of optical axis from nadir (-Z): angle between axis and straight down.
ALPHA_FROM_NADIR_DEG = 30

# Simulation
INITIAL_PX = 10.0
INITIAL_PY = 50.0
INITIAL_YAW_RAD = 0.0
INITIAL_T_SIM = 0.0
# Long horizon for holding last command (seconds)
DEFAULT_HISTORY_T_MAX = 1_000_000.0

# Trajectory when no CSV: "snake" (boustrophedon) or "constant" (straight + spin)
TRAJECTORY = os.environ.get("ROOMBA_TRAJECTORY", "snake").strip().lower()
SNAKE_MARGIN_M = 1.0
SNAKE_ROW_SPACING_M = 20.0
SNAKE_SPEED_M_S = 40.0
# Snake: |yaw_rate| while moving; sign alternates each horizontal line (vertical uses next line's sign)
YAW_RATE_MAG_RAD_S = float(os.environ.get("ROOMBA_YAW_RATE", str(math.pi*0.01)))
# +1 or -1: rotation direction for the first horizontal line (CCW positive about +Z)
SNAKE_YAW_INITIAL_SIGN = float(os.environ.get("ROOMBA_YAW_SIGN", "-1.0"))
if SNAKE_YAW_INITIAL_SIGN not in (-1.0, 1.0):
    SNAKE_YAW_INITIAL_SIGN = 1.0 if SNAKE_YAW_INITIAL_SIGN >= 0.0 else -1.0

# Constant trajectory (ROOMBA_TRAJECTORY=constant): signed yaw rate vs time (CSV still full control)
CONSTANT_YAW_RATE_RAD_S = float(os.environ.get("ROOMBA_CONSTANT_YAW", str(math.pi)))

# Optional CSV path (overrides trajectory mode when set)
HISTORY_CSV_PATH = os.environ.get("ROOMBA_HISTORY_CSV")


def initial_pose() -> tuple[float, float]:
    """Start position; snake begins at bottom-left inside margin."""
    if TRAJECTORY == "snake" and not HISTORY_CSV_PATH:
        return (float(SNAKE_MARGIN_M), float(SNAKE_MARGIN_M))
    return (float(INITIAL_PX), float(INITIAL_PY))

# Cell color value in [0,1]: 0=red, 1=blue. In FOV: +VIEWSLOPE*dt/s; off FOV: *= exp(-dt/tau)
VIEWSLOPE = float(os.environ.get("ROOMBA_VIEWSLOPE", "3.0"))
DEGRADE_TAU_S = float(os.environ.get("ROOMBA_DEGRADE_TAU", "8.0"))
# Only cells within this horizontal range (m) from the drone accumulate; use "inf" for no limit
_ACCUM = os.environ.get("ROOMBA_ACCUM_MAX_DIST", "25.0").strip().lower()
COLOR_ACCUM_MAX_DIST_M = float("inf") if _ACCUM in ("", "inf", "none") else float(_ACCUM)

# Display
CELL_PIXELS = 4
FPS = 60
