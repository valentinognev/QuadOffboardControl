#!/usr/bin/env python3
"""Recolor multi-SITL iris visuals to Observation Board drone colors at spawn.

Keep DRONE_COLORS in sync with:
  ObservationBoard/web/frontend/src/utils/droneColors.ts

Mapping: iris_N / PX4 -i N → OB drone id N → palette[(N - 1) % len].
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

# Keep in sync with ObservationBoard/web/frontend/src/utils/droneColors.ts
DRONE_COLORS = [
    "#FF2D00",  # Bright Red
    "#008CFF",  # Azure Blue
    "#00D615",  # Bright Green
    "#FFD200",  # Golden Yellow
    "#BE00FF",  # Electric Purple
    "#FF00D9",  # Magenta
    "#00E0E0",  # Cyan
    "#FF8800",  # Orange
    "#8FDF00",  # Lime
]

_MATERIAL_SCRIPT_RE = re.compile(
    r"<material>\s*<script>.*?</script>\s*</material>",
    re.DOTALL | re.IGNORECASE,
)


def color_for_instance(instance_n: int) -> str:
    if instance_n < 1:
        raise SystemExit(f"inject_iris_colors: instance_n must be >= 1, got {instance_n}")
    return DRONE_COLORS[(instance_n - 1) % len(DRONE_COLORS)]


def hex_to_rgba(hex_color: str) -> tuple[float, float, float, float]:
    h = hex_color.strip().lstrip("#")
    if len(h) != 6:
        raise SystemExit(f"inject_iris_colors: bad hex color {hex_color!r}")
    r = int(h[0:2], 16) / 255.0
    g = int(h[2:4], 16) / 255.0
    b = int(h[4:6], 16) / 255.0
    return (r, g, b, 1.0)


def rgba_for_instance(instance_n: int) -> tuple[float, float, float, float]:
    return hex_to_rgba(color_for_instance(instance_n))


def material_xml(rgba: tuple[float, float, float, float]) -> str:
    r, g, b, a = rgba
    return (
        "<material>\n"
        f"          <ambient>{r:.6f} {g:.6f} {b:.6f} {a:.6f}</ambient>\n"
        f"          <diffuse>{r:.6f} {g:.6f} {b:.6f} {a:.6f}</diffuse>\n"
        f"          <specular>0.100000 0.100000 0.100000 1.000000</specular>\n"
        f"          <emissive>0.000000 0.000000 0.000000 1.000000</emissive>\n"
        "        </material>"
    )


def inject(sdf_path: Path, instance_n: int) -> int:
    """Replace Gazebo script materials with OB color. Returns replacement count."""
    try:
        text = sdf_path.read_text(encoding="utf-8")
    except OSError as e:
        raise SystemExit(f"inject_iris_colors: cannot read {sdf_path}: {e}") from e

    replacement = material_xml(rgba_for_instance(instance_n))
    new_text, n = _MATERIAL_SCRIPT_RE.subn(replacement, text)
    if n == 0:
        print(f"inject_iris_colors: no script materials in {sdf_path} (noop)")
        return 0

    try:
        sdf_path.write_text(new_text, encoding="utf-8")
    except OSError as e:
        raise SystemExit(f"inject_iris_colors: cannot write {sdf_path}: {e}") from e

    print(
        f"inject_iris_colors: instance={instance_n} "
        f"color={color_for_instance(instance_n)} replaced={n} in {sdf_path}"
    )
    return n


def main() -> None:
    if len(sys.argv) != 3:
        raise SystemExit(f"usage: {sys.argv[0]} <iris.sdf> <instance_n>")
    sdf = Path(sys.argv[1])
    try:
        instance_n = int(sys.argv[2])
    except ValueError as e:
        raise SystemExit(
            f"inject_iris_colors: instance_n must be int, got {sys.argv[2]!r}"
        ) from e
    inject(sdf, instance_n)


if __name__ == "__main__":
    main()
