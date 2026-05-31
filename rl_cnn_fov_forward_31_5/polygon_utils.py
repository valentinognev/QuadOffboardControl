from __future__ import annotations

import math
import numpy as np

DEFAULT_POLYGON_NUM_VERTICES = 12
DEFAULT_POLYGON_TARGET_AREA_M2 = 900.0


def polygon_area(vertices: np.ndarray) -> float:
    v = np.asarray(vertices, dtype=np.float64)
    x = v[:, 0]
    y = v[:, 1]
    return 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def ensure_ccw(vertices: np.ndarray) -> np.ndarray:
    v = np.asarray(vertices, dtype=np.float64)
    signed = np.dot(v[:, 0], np.roll(v[:, 1], -1)) - np.dot(v[:, 1], np.roll(v[:, 0], -1))
    return v if signed >= 0 else v[::-1].copy()


def _subdivide_polygon_edges_to_vertex_count(vertices: np.ndarray, n: int) -> np.ndarray:
    v = ensure_ccw(vertices)
    while len(v) < n:
        edges = np.roll(v, -1, axis=0) - v
        d2 = np.sum(edges * edges, axis=1)
        i = int(np.argmax(d2))
        j = (i + 1) % len(v)
        mid = 0.5 * (v[i] + v[j])
        v = np.insert(v, i + 1, mid, axis=0)
    return v


def _resample_polygon_vertex_count(vertices: np.ndarray, n: int) -> np.ndarray:
    v = ensure_ccw(vertices)
    if len(v) == n:
        return v.copy()
    nxt = np.roll(v, -1, axis=0)
    edges = nxt - v
    lens = np.linalg.norm(edges, axis=1)
    perim = float(np.sum(lens))
    if perim <= 1e-12:
        return np.repeat(v[:1], n, axis=0)
    cum = np.concatenate([[0.0], np.cumsum(lens)])
    out = np.zeros((n, 2), dtype=np.float64)
    for k in range(n):
        s = (k + 0.5) * perim / n
        i = int(np.searchsorted(cum, s, side='right') - 1)
        i = max(0, min(len(v) - 1, i))
        seg_len = cum[i + 1] - cum[i]
        if seg_len <= 1e-12:
            out[k] = v[i]
        else:
            t = (s - cum[i]) / seg_len
            out[k] = v[i] + t * (v[(i + 1) % len(v)] - v[i])
    return out


def _finalize_polygon(vertices: np.ndarray, num_vertices: int, target_area: float) -> np.ndarray:
    v = ensure_ccw(vertices)
    if len(v) < num_vertices:
        v = _subdivide_polygon_edges_to_vertex_count(v, num_vertices)
    elif len(v) > num_vertices:
        v = _resample_polygon_vertex_count(v, num_vertices)
    area = polygon_area(v)
    if area > 1e-12:
        v = v * math.sqrt(target_area / area)
    centroid = np.mean(v, axis=0)
    angles = np.arctan2(v[:, 1] - centroid[1], v[:, 0] - centroid[0])
    start_idx = int(np.argmin(angles))
    return ensure_ccw(np.roll(v, -start_idx, axis=0))


def generate_rectilinear_polygon(
    num_rectangles: int = 6,
    num_vertices_target: int = DEFAULT_POLYGON_NUM_VERTICES,
    target_area: float = DEFAULT_POLYGON_TARGET_AREA_M2,
    rng: np.random.Generator | None = None,
    show_plot: bool = False,
) -> np.ndarray:
    import cv2

    rng = rng or np.random.default_rng()
    canvas_size = 500
    increments = 20
    max_size_cells = 8
    min_size_cells = 2
    margin_cells = max_size_cells + 1

    def randint(lo: int, hi: int) -> int:
        return int(rng.integers(lo, hi + 1))

    boxes: list[list[int]] = []
    for _ in range(num_rectangles):
        placed = False
        for _attempt in range(2000):
            w = randint(min_size_cells, max_size_cells) * increments
            h = randint(min_size_cells, max_size_cells) * increments
            cx = randint(margin_cells, canvas_size // increments - margin_cells) * increments
            cy = randint(margin_cells, canvas_size // increments - margin_cells) * increments
            if not boxes:
                boxes.append([cx, cy, w, h])
                placed = True
                break
            touching = any(abs(cx - bx) <= (w + bw) / 2.0 and abs(cy - by) <= (h + bh) / 2.0 for bx, by, bw, bh in boxes)
            if touching:
                boxes.append([cx, cy, w, h])
                placed = True
                break
        if not placed:
            bx, by, bw, bh = boxes[randint(0, len(boxes) - 1)]
            w = randint(min_size_cells, max_size_cells) * increments
            h = randint(min_size_cells, max_size_cells) * increments
            cx = int(np.clip(bx + randint(-max_size_cells, max_size_cells) * increments, margin_cells * increments, (canvas_size // increments - margin_cells) * increments))
            cy = int(np.clip(by + randint(-max_size_cells, max_size_cells) * increments, margin_cells * increments, (canvas_size // increments - margin_cells) * increments))
            boxes.append([cx, cy, w, h])

    mask = np.zeros((canvas_size, canvas_size), dtype=np.uint8)
    for cx, cy, w, h in boxes:
        x0 = max(0, int(cx - w // 2))
        y0 = max(0, int(cy - h // 2))
        x1 = min(canvas_size - 1, int(cx + w // 2))
        y1 = min(canvas_size - 1, int(cy + h // 2))
        mask[y0:y1, x0:x1] = 255

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        side = math.sqrt(target_area)
        half = side / 2.0
        vertices = np.array([[-half, -half], [half, -half], [half, half], [-half, half]], dtype=np.float64)
        return _finalize_polygon(vertices, num_vertices_target, target_area)

    contour = max(contours, key=cv2.contourArea)
    perimeter = cv2.arcLength(contour, True)
    eps_lo, eps_hi = 0.0, perimeter * 0.5
    best = contour
    for _ in range(30):
        eps_mid = 0.5 * (eps_lo + eps_hi)
        approx = cv2.approxPolyDP(contour, eps_mid, True)
        n = len(approx)
        if n == num_vertices_target:
            best = approx
            break
        if n > num_vertices_target:
            eps_lo = eps_mid
        else:
            eps_hi = eps_mid
            if n >= max(4, num_vertices_target - 2):
                best = approx

    approx = best.reshape(-1, 2).astype(np.float64)
    approx[:, 1] = canvas_size - approx[:, 1]
    approx -= np.mean(approx, axis=0)
    area = polygon_area(approx)
    if area > 1e-12:
        approx *= math.sqrt(target_area / area)
    vertices = _finalize_polygon(approx, num_vertices_target, target_area)

    if show_plot:
        import matplotlib.pyplot as plt
        closed = np.vstack([vertices, vertices[0:1]])
        plt.figure(figsize=(6, 6))
        plt.plot(closed[:, 0], closed[:, 1], 'k-')
        plt.fill(closed[:, 0], closed[:, 1], alpha=0.25)
        plt.gca().set_aspect('equal')
        plt.grid(True, alpha=0.3)
        plt.show()

    return vertices


def generate_concave_polygon(
    num_vertices: int = DEFAULT_POLYGON_NUM_VERTICES,
    target_area: float = DEFAULT_POLYGON_TARGET_AREA_M2,
    rng: np.random.Generator | None = None,
    show_plot: bool = False,
) -> np.ndarray:
    rng = rng or np.random.default_rng()
    return generate_rectilinear_polygon(
        num_rectangles=max(4, int(rng.integers(5, 8))),
        num_vertices_target=num_vertices,
        target_area=target_area,
        rng=rng,
        show_plot=show_plot,
    )


def point_in_polygon(point: np.ndarray, vertices: np.ndarray) -> bool:
    x, y = float(point[0]), float(point[1])
    verts = np.asarray(vertices, dtype=np.float64)
    inside = False
    j = len(verts) - 1
    for i in range(len(verts)):
        xi, yi = verts[i]
        xj, yj = verts[j]
        intersects = ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi)
        if intersects:
            inside = not inside
        j = i
    return inside


def points_in_polygon(points: np.ndarray, vertices: np.ndarray) -> np.ndarray:
    pts = np.asarray(points, dtype=np.float64)
    verts = np.asarray(vertices, dtype=np.float64)
    x = pts[..., 0]
    y = pts[..., 1]
    xi = verts[:, 0]
    yi = verts[:, 1]
    xj = np.roll(xi, 1)
    yj = np.roll(yi, 1)
    cond = ((yi > y[..., None]) != (yj > y[..., None]))
    x_inter = (xj - xi) * (y[..., None] - yi) / (yj - yi + 1e-12) + xi
    crossings = cond & (x[..., None] < x_inter)
    return np.sum(crossings, axis=-1) % 2 == 1


def random_point_in_polygon(vertices: np.ndarray, rng: np.random.Generator | None = None) -> np.ndarray:
    rng = rng or np.random.default_rng()
    verts = np.asarray(vertices, dtype=np.float64)
    lo = verts.min(axis=0)
    hi = verts.max(axis=0)
    for _ in range(10000):
        pt = rng.uniform(lo, hi)
        if point_in_polygon(pt, verts):
            return pt.astype(np.float32)
    return np.mean(verts, axis=0).astype(np.float32)


def distance_to_polygon_edge(point: np.ndarray, vertices: np.ndarray) -> float:
    p = np.asarray(point, dtype=np.float64)
    v = np.asarray(vertices, dtype=np.float64)
    nxt = np.roll(v, -1, axis=0)
    ab = nxt - v  # (E, 2)
    den = np.sum(ab * ab, axis=1)  # (E,)
    ap = p[None, :] - v  # (E, 2)
    t = np.clip(np.sum(ap * ab, axis=1) / np.maximum(den, 1e-12), 0.0, 1.0)
    closest = v + t[:, None] * ab  # (E, 2)
    dists = np.linalg.norm(p[None, :] - closest, axis=1)
    return float(np.min(dists))


def distance_to_polygon_edge_batch(points: np.ndarray, vertices: np.ndarray) -> np.ndarray:
    """Batch version: points (N,2) → distances (N,)."""
    pts = np.asarray(points, dtype=np.float64)  # (N, 2)
    v = np.asarray(vertices, dtype=np.float64)  # (E, 2)
    nxt = np.roll(v, -1, axis=0)
    ab = nxt - v  # (E, 2)
    den = np.sum(ab * ab, axis=1)  # (E,)
    # pts[:, None, :] - v[None, :, :] → (N, E, 2)
    ap = pts[:, None, :] - v[None, :, :]
    t = np.clip(np.sum(ap * ab[None, :, :], axis=2) / np.maximum(den[None, :], 1e-12), 0.0, 1.0)  # (N, E)
    closest = v[None, :, :] + t[:, :, None] * ab[None, :, :]  # (N, E, 2)
    dists = np.linalg.norm(pts[:, None, :] - closest, axis=2)  # (N, E)
    return np.min(dists, axis=1).astype(np.float32)


def polygon_bbox(vertices: np.ndarray) -> tuple[float, float, float, float]:
    v = np.asarray(vertices, dtype=np.float64)
    return float(v[:, 0].min()), float(v[:, 0].max()), float(v[:, 1].min()), float(v[:, 1].max())


def sample_separated_points_in_polygon(vertices: np.ndarray, count: int, min_edge_distance: float, min_pair_distance: float, rng: np.random.Generator | None = None) -> np.ndarray:
    rng = rng or np.random.default_rng()
    pts: list[np.ndarray] = []
    relaxed = min_pair_distance
    while len(pts) < count:
        for _ in range(10000):
            pt = random_point_in_polygon(vertices, rng)
            if distance_to_polygon_edge(pt, vertices) < min_edge_distance:
                continue
            if any(np.linalg.norm(pt - q) < relaxed for q in pts):
                continue
            pts.append(pt)
            break
        else:
            relaxed *= 0.9
            if relaxed < 0.5:
                pts.append(random_point_in_polygon(vertices, rng))
    return np.asarray(pts[:count], dtype=np.float32)
