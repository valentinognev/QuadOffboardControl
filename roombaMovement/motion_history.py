"""Time-series velocity and yaw-rate commands with linear interpolation."""

from __future__ import annotations

import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Union

import numpy as np

from config import (
    CONSTANT_YAW_RATE_RAD_S,
    DEFAULT_HISTORY_T_MAX,
    PATCH_SIZE_M,
    SNAKE_MARGIN_M,
    SNAKE_ROW_SPACING_M,
    SNAKE_SPEED_M_S,
    SNAKE_YAW_INITIAL_SIGN,
    TRAJECTORY,
    YAW_RATE_MAG_RAD_S,
)


@dataclass(frozen=True)
class MotionHistory:
    """Strictly increasing time stamps; aligned vx, vy, yaw_rate (world frame, rad/s)."""

    t: np.ndarray
    vx: np.ndarray
    vy: np.ndarray
    yaw_rate: np.ndarray
    period: float | None = None

    def __post_init__(self) -> None:
        t = np.asarray(self.t, dtype=np.float64).ravel()
        vx = np.asarray(self.vx, dtype=np.float64).ravel()
        vy = np.asarray(self.vy, dtype=np.float64).ravel()
        yr = np.asarray(self.yaw_rate, dtype=np.float64).ravel()
        n = t.size
        if n == 0:
            raise ValueError("MotionHistory: empty arrays")
        if vx.size != n or vy.size != n or yr.size != n:
            raise ValueError("MotionHistory: t, vx, vy, yaw_rate must have same length")
        if n >= 2 and np.any(np.diff(t) <= 0):
            raise ValueError("MotionHistory: t must be strictly increasing")
        object.__setattr__(self, "t", t)
        object.__setattr__(self, "vx", vx)
        object.__setattr__(self, "vy", vy)
        object.__setattr__(self, "yaw_rate", yr)
        per = self.period
        if per is not None and per <= 0.0:
            raise ValueError("MotionHistory: period must be positive or None")
        object.__setattr__(self, "period", float(per) if per is not None else None)

    def sample(self, t_query: float) -> tuple[float, float, float]:
        """Linear interpolation; hold first/last outside range. Cyclic if period is set."""
        if self.t.size == 1:
            return float(self.vx[0]), float(self.vy[0]), float(self.yaw_rate[0])
        tq = float(t_query)
        if self.period is not None and self.period > 0.0:
            tq = tq % self.period
        vx = float(np.interp(tq, self.t, self.vx))
        vy = float(np.interp(tq, self.t, self.vy))
        omega = float(np.interp(tq, self.t, self.yaw_rate))
        return vx, vy, omega


def default_history() -> MotionHistory:
    """Constant flight: vx=3, vy=0, yaw from CONSTANT_YAW_RATE_RAD_S."""
    t0 = 0.0
    t1 = DEFAULT_HISTORY_T_MAX
    yr = float(CONSTANT_YAW_RATE_RAD_S)
    return MotionHistory(
        t=np.array([t0, t1], dtype=np.float64),
        vx=np.array([3.0, 3.0], dtype=np.float64),
        vy=np.array([0.0, 0.0], dtype=np.float64),
        yaw_rate=np.array([yr, yr], dtype=np.float64),
        period=None,
    )


def _integrate_segments_xy(
    segs: list[tuple[float, float, float, float]],
    x0: float,
    y0: float,
) -> tuple[float, float]:
    px, py = x0, y0
    for vx, vy, d, _ in segs:
        px += vx * d
        py += vy * d
    return px, py


def _knots_from_segments_loop_closed(
    segs: list[tuple[float, float, float, float]],
    corner_eps: float,
    loop_vx: float,
    loop_vy: float,
    loop_yr: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, float]:
    if not segs:
        raise ValueError("segments empty")
    ts: list[float] = []
    vxs: list[float] = []
    vys: list[float] = []
    yrs: list[float] = []

    vx_cur, vy_cur, d0, yr_cur = segs[0]
    ts.append(0.0)
    vxs.append(vx_cur)
    vys.append(vy_cur)
    yrs.append(yr_cur)
    t_end = d0

    for k in range(1, len(segs)):
        vx_next, vy_next, dk, yr_next = segs[k]
        ts.append(t_end - corner_eps)
        vxs.append(vx_cur)
        vys.append(vy_cur)
        yrs.append(yr_cur)
        ts.append(t_end)
        vxs.append(vx_next)
        vys.append(vy_next)
        yrs.append(yr_next)
        vx_cur, vy_cur, yr_cur = vx_next, vy_next, yr_next
        t_end += dk

    ts.append(t_end - corner_eps)
    vxs.append(vx_cur)
    vys.append(vy_cur)
    yrs.append(yr_cur)
    ts.append(t_end)
    vxs.append(loop_vx)
    vys.append(loop_vy)
    yrs.append(loop_yr)
    period = t_end
    return (
        np.array(ts, dtype=np.float64),
        np.array(vxs, dtype=np.float64),
        np.array(vys, dtype=np.float64),
        np.array(yrs, dtype=np.float64),
        period,
    )


def snake_history(
    patch_m: float = PATCH_SIZE_M,
    margin_m: float = SNAKE_MARGIN_M,
    row_spacing_m: float = SNAKE_ROW_SPACING_M,
    speed_m_s: float = SNAKE_SPEED_M_S,
    yaw_rate_mag_rad_s: float | None = None,
    initial_line_yaw_sign: float | None = None,
) -> MotionHistory:
    """
    Cyclic boustrophedon on [margin, patch-margin]^2, return to (margin, margin).
    Yaw sign alternates each horizontal line; vertical uses next line's sign.
    """
    v = float(speed_m_s)
    if v <= 0.0:
        raise ValueError("snake_history: speed must be positive")
    lo = float(margin_m)
    hi = float(patch_m - margin_m)
    if hi <= lo:
        raise ValueError("snake_history: margin too large for patch")
    dy_step = float(row_spacing_m)
    if dy_step <= 0.0:
        raise ValueError("snake_history: row_spacing must be positive")

    yaw_mag = float(YAW_RATE_MAG_RAD_S if yaw_rate_mag_rad_s is None else yaw_rate_mag_rad_s)
    line_sign = float(SNAKE_YAW_INITIAL_SIGN if initial_line_yaw_sign is None else initial_line_yaw_sign)
    line_sign = 1.0 if line_sign >= 0.0 else -1.0

    corner_eps = 1e-4
    segs: list[tuple[float, float, float, float]] = []
    y = lo
    going_right = True

    while y <= hi + 1e-9:
        x_lo, x_hi = lo, hi
        xh0, xh1 = (x_lo, x_hi) if going_right else (x_hi, x_lo)
        horiz_len = abs(xh1 - xh0)
        if horiz_len > 0.0:
            vx_h = v if xh1 > xh0 else -v
            yr_h = line_sign * yaw_mag
            segs.append((vx_h, 0.0, horiz_len / v, yr_h))

        if y >= hi - 1e-9:
            break

        y_next = min(y + dy_step, hi)
        vert_len = y_next - y
        if vert_len > 0.0:
            next_sign = -line_sign
            yr_v = next_sign * yaw_mag
            segs.append((0.0, v, vert_len / v, yr_v))
            line_sign = next_sign
        y = y_next
        going_right = not going_right

    if not segs:
        segs.append((0.0, 0.0, 0.0, 0.0))

    vx0, vy0, _, yr0 = segs[0]
    ex, ey = _integrate_segments_xy(segs, lo, lo)
    dx = lo - ex
    dy = lo - ey
    dist = math.hypot(dx, dy)
    if dist > 1e-6:
        vx_r = dx / dist * v
        vy_r = dy / dist * v
        segs.append((vx_r, vy_r, dist / v, yr0))

    ts, vxs, vys, yrs, period = _knots_from_segments_loop_closed(
        segs, corner_eps, vx0, vy0, yr0
    )

    return MotionHistory(
        t=ts,
        vx=vxs,
        vy=vys,
        yaw_rate=yrs,
        period=period,
    )


def load_csv(path: Union[str, Path]) -> MotionHistory:
    path = Path(path)
    rows: list[tuple[float, float, float, float]] = []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError("CSV: missing header")
        fields = {h.strip().lower(): h for h in reader.fieldnames}
        for key in ("t", "vx", "vy", "yaw_rate"):
            if key not in fields:
                raise ValueError(f"CSV: need column '{key}' (case-insensitive)")
        for row in reader:
            rows.append(
                (
                    float(row[fields["t"]]),
                    float(row[fields["vx"]]),
                    float(row[fields["vy"]]),
                    float(row[fields["yaw_rate"]]),
                )
            )
    if not rows:
        raise ValueError("CSV: no data rows")
    t, vx, vy, yr = zip(*rows)
    return MotionHistory(
        t=np.array(t, dtype=np.float64),
        vx=np.array(vx, dtype=np.float64),
        vy=np.array(vy, dtype=np.float64),
        yaw_rate=np.array(yr, dtype=np.float64),
        period=None,
    )


def load_json(path: Union[str, Path]) -> MotionHistory:
    path = Path(path)
    with path.open() as f:
        data = json.load(f)
    per = data.get("period")
    return MotionHistory(
        t=np.array(data["t"], dtype=np.float64),
        vx=np.array(data["vx"], dtype=np.float64),
        vy=np.array(data["vy"], dtype=np.float64),
        yaw_rate=np.array(data["yaw_rate"], dtype=np.float64),
        period=float(per) if per is not None else None,
    )


def load_history(path: Union[str, Path, None]) -> MotionHistory:
    if path is None:
        if TRAJECTORY == "constant":
            return default_history()
        return snake_history()
    p = Path(path)
    suf = p.suffix.lower()
    if suf == ".csv":
        return load_csv(p)
    if suf == ".json":
        return load_json(p)
    raise ValueError(f"Unknown history file type: {suf}")


def history_from_columns(
    t: Iterable[float],
    vx: Iterable[float],
    vy: Iterable[float],
    yaw_rate: Iterable[float],
    period: float | None = None,
) -> MotionHistory:
    return MotionHistory(
        t=np.array(list(t), dtype=np.float64),
        vx=np.array(list(vx), dtype=np.float64),
        vy=np.array(list(vy), dtype=np.float64),
        yaw_rate=np.array(list(yaw_rate), dtype=np.float64),
        period=period,
    )
