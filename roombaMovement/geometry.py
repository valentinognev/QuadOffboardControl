"""Camera frustum intersection with ground plane and grid marking."""

from __future__ import annotations

import math
from typing import Optional

import numpy as np

from config import ALT_M, CELL_SIZE_M, GRID_N, PATCH_SIZE_M

# Max horizontal reach for footprint (m); clips grazing frustum rays.
_MAX_REACH_M = 3.0 * PATCH_SIZE_M


def optical_axis_world(yaw_rad: float, alpha_from_nadir_rad: float) -> np.ndarray:
    """
    Unit optical axis in world ENU: Z up, yaw about +Z.
    Body forward +X tilted alpha from nadir in XZ body plane, then yaw.
    """
    sa = math.sin(alpha_from_nadir_rad)
    ca = math.cos(alpha_from_nadir_rad)
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    # Body (sin a, 0, -cos a) -> world
    return np.array([sa * c, sa * s, -ca], dtype=np.float64)


def _world_camera_frame(
    d: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    OpenCV-style camera in world: optical axis d (forward), r = X (right),
    dn = Y (image down). Rays: normalize(d + sx*tan(fx/2)*r + sy*tan(fy/2)*dn).
    """
    ez = np.array([0.0, 0.0, 1.0], dtype=np.float64)
    r = np.cross(d, ez)
    rn = np.linalg.norm(r)
    if rn < 1e-12:
        r = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    else:
        r = r / rn
    dn = np.cross(d, r)
    dn = dn / np.linalg.norm(dn)
    return d, r, dn


def _min_abs_dz_for_alt(alt: float) -> float:
    """Steepest |dz| for a ray from height alt that reaches _MAX_REACH_M horizontally."""
    return alt / math.sqrt(_MAX_REACH_M * _MAX_REACH_M + alt * alt)


def _ensure_downward(direction: np.ndarray, min_abs_dz: float) -> np.ndarray:
    """Unit direction with z <= -min_abs_dz (clips grazing rays to max ground reach)."""
    v = np.asarray(direction, dtype=np.float64).copy()
    if v[2] > -min_abs_dz:
        v[2] = -min_abs_dz
    n = np.linalg.norm(v)
    if n < 1e-12:
        return np.array([0.0, 0.0, -1.0], dtype=np.float64)
    return v / n


def ray_ground_intersection(
    origin: np.ndarray,
    direction: np.ndarray,
    ground_z: float = 0.0,
    alt_m: float = ALT_M,
) -> Optional[np.ndarray]:
    """Return XY hit on z=ground_z; nudges grazing rays using max-reach limit."""
    d = _ensure_downward(direction, _min_abs_dz_for_alt(alt_m))
    oz = float(origin[2])
    dz = float(d[2])
    t = (ground_z - oz) / dz
    if t <= 0.0:
        return None
    p = origin + t * d
    return p.astype(np.float64)


def _order_vertices_ccw(poly_xy: np.ndarray) -> np.ndarray:
    """Sort polygon vertices CCW around centroid (fixes bow-ties from corner order)."""
    c = poly_xy.mean(axis=0)
    ang = np.arctan2(poly_xy[:, 1] - c[1], poly_xy[:, 0] - c[0])
    return poly_xy[np.argsort(ang)]


def frustum_ground_quad(
    cam_xy: np.ndarray,
    alt: float,
    yaw_rad: float,
    fov_x_rad: float,
    fov_y_rad: float,
    alpha_from_nadir_rad: float,
) -> Optional[np.ndarray]:
    """
    Four ground vertices (4, 2) in CCW order (sorted by polar angle), or None.
    """
    origin = np.array([cam_xy[0], cam_xy[1], alt], dtype=np.float64)
    d = optical_axis_world(yaw_rad, alpha_from_nadir_rad)
    _, r, dn = _world_camera_frame(d)

    half_w = math.tan(0.5 * fov_x_rad)
    half_h = math.tan(0.5 * fov_y_rad)
    corners = []
    for sx, sy in ((-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)):
        v = d + sx * half_w * r + sy * half_h * dn
        vn = np.linalg.norm(v)
        if vn < 1e-12:
            return None
        v = v / vn
        hit = ray_ground_intersection(origin, v, 0.0, alt)
        if hit is None:
            return None
        corners.append(hit[:2])
    raw = np.stack(corners, axis=0)
    return _order_vertices_ccw(raw)


def points_in_convex_quad(points_xy: np.ndarray, quad_xy: np.ndarray) -> np.ndarray:
    """
    points_xy: (N, 2), quad_xy: (4, 2) CCW in world XY.
    Half-plane test: interior lies to the left of each directed edge.
    """
    n = points_xy.shape[0]
    if n == 0:
        return np.zeros((0,), dtype=bool)
    inside = np.ones(n, dtype=bool)
    eps = 1e-9
    for i in range(4):
        a = quad_xy[i]
        b = quad_xy[(i + 1) % 4]
        edge = b - a
        rel = points_xy - a
        c = edge[0] * rel[:, 1] - edge[1] * rel[:, 0]
        inside &= c >= -eps
    return inside


def cell_centers_xy() -> np.ndarray:
    """Shape (GRID_N, GRID_N, 2) — cell (i,j) center in world XY."""
    half = 0.5 * CELL_SIZE_M
    xs = np.arange(GRID_N, dtype=np.float64) * CELL_SIZE_M + half
    ys = np.arange(GRID_N, dtype=np.float64) * CELL_SIZE_M + half
    gx, gy = np.meshgrid(xs, ys, indexing="ij")
    return np.stack([gx, gy], axis=-1)


_CENTERS_FLAT: Optional[np.ndarray] = None


def _centers_flat() -> np.ndarray:
    global _CENTERS_FLAT
    if _CENTERS_FLAT is None:
        _CENTERS_FLAT = cell_centers_xy().reshape(-1, 2)
    return _CENTERS_FLAT


def mark_footprint(
    visited: np.ndarray,
    cam_xy: np.ndarray,
    yaw_rad: float,
    fov_x_rad: float,
    fov_y_rad: float,
    alpha_from_nadir_rad: float,
    cell_color_01: np.ndarray | None = None,
    color_slope_dt: float = 0.0,
    off_view_decay: float = 1.0,
    max_accum_dist_m: float = float("inf"),
) -> None:
    """
    OR footprint into visited[i,j] bool (mutates).
    If cell_color_01 is float64 grid: per-cell value in [0,1] (0=red, 1=blue).
    Accumulation (increase) only if in footprint, in patch, and within max_accum_dist_m
    horizontally of cam_xy. All other cells multiply by off_view_decay.
    visited uses full footprint (no distance limit).
    """
    quad = frustum_ground_quad(
        cam_xy, ALT_M, yaw_rad, fov_x_rad, fov_y_rad, alpha_from_nadir_rad
    )
    if quad is None:
        if cell_color_01 is not None and off_view_decay < 1.0:
            cell_color_01[:] *= off_view_decay
        return
    pts = _centers_flat()
    inside = points_in_convex_quad(pts, quad)
    in_patch = (
        (pts[:, 0] >= 0.0)
        & (pts[:, 0] <= PATCH_SIZE_M)
        & (pts[:, 1] >= 0.0)
        & (pts[:, 1] <= PATCH_SIZE_M)
    )
    mask_fov = inside & in_patch
    idx = np.arange(GRID_N * GRID_N)
    i_idx = idx // GRID_N
    j_idx = idx % GRID_N
    mi_vis = i_idx[mask_fov]
    mj_vis = j_idx[mask_fov]
    visited[mi_vis, mj_vis] = True

    if cell_color_01 is not None:
        cx = float(cam_xy[0])
        cy = float(cam_xy[1])
        dx = pts[:, 0] - cx
        dy = pts[:, 1] - cy
        d2 = dx * dx + dy * dy
        if math.isfinite(max_accum_dist_m) and max_accum_dist_m > 0.0:
            r2 = max_accum_dist_m * max_accum_dist_m
            mask_accum_flat = mask_fov & (d2 <= r2)
        else:
            mask_accum_flat = mask_fov
        mi_a = i_idx[mask_accum_flat]
        mj_a = j_idx[mask_accum_flat]
        mask_accum_2d = np.zeros((GRID_N, GRID_N), dtype=bool)
        mask_accum_2d[mi_a, mj_a] = True
        cc = cell_color_01
        cc[~mask_accum_2d] *= off_view_decay
        inc = float(color_slope_dt)
        if inc > 0.0:
            cc[mask_accum_2d] = np.minimum(1.0, cc[mask_accum_2d] + inc)
