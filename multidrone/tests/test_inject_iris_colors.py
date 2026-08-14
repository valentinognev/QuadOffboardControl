from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from inject_iris_colors import (  # noqa: E402
    DRONE_COLORS,
    color_for_instance,
    hex_to_rgba,
    inject,
    material_xml,
)

# Keep in sync with ObservationBoard/web/frontend/src/utils/droneColors.ts
OB_COLORS = [
    "#FF2D00",
    "#008CFF",
    "#00D615",
    "#FFD200",
    "#BE00FF",
    "#FF00D9",
    "#00E0E0",
    "#FF8800",
    "#8FDF00",
]

IRIS_VISUALS = """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='iris_1'>
    <link name='base_link'>
      <visual name='base_link_inertia_visual'>
        <material>
          <script>
            <name>Gazebo/DarkGrey</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
        </material>
      </visual>
    </link>
    <link name='rotor_0'>
      <visual name='rotor_0_visual'>
        <material>
          <script>
            <name>Gazebo/Blue</name>
            <uri>file://media/materials/scripts/gazebo.material</uri>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def _write(tmp_path: Path, text: str) -> Path:
    sdf = tmp_path / "iris_1.sdf"
    sdf.write_text(text, encoding="utf-8")
    return sdf


def test_palette_matches_observation_board():
    assert DRONE_COLORS == OB_COLORS


def test_color_for_instance_maps_n_to_ob_id_n():
    assert color_for_instance(1) == "#FF2D00"
    assert color_for_instance(2) == "#008CFF"
    assert color_for_instance(10) == "#FF2D00"


def test_hex_to_rgba_red():
    r, g, b, a = hex_to_rgba("#FF2D00")
    assert a == 1.0
    assert abs(r - 1.0) < 1e-9
    assert abs(g - 0x2D / 255.0) < 1e-9
    assert abs(b - 0.0) < 1e-9


def test_inject_replaces_all_script_materials(tmp_path: Path):
    sdf = _write(tmp_path, IRIS_VISUALS)
    n = inject(sdf, instance_n=1)
    text = sdf.read_text(encoding="utf-8")
    assert n == 2
    assert "Gazebo/DarkGrey" not in text
    assert "Gazebo/Blue" not in text
    assert "<script>" not in text
    rgba = hex_to_rgba("#FF2D00")
    block = material_xml(rgba)
    assert text.count("<ambient>") == 2
    assert block.splitlines()[0].strip() in text


def test_inject_idempotent(tmp_path: Path):
    sdf = _write(tmp_path, IRIS_VISUALS)
    inject(sdf, instance_n=1)
    once = sdf.read_text(encoding="utf-8")
    n2 = inject(sdf, instance_n=1)
    assert sdf.read_text(encoding="utf-8") == once
    assert n2 == 0


def test_inject_instance_2_uses_blue(tmp_path: Path):
    sdf = _write(tmp_path, IRIS_VISUALS)
    inject(sdf, instance_n=2)
    text = sdf.read_text(encoding="utf-8")
    r, g, b, a = hex_to_rgba("#008CFF")
    assert f"<diffuse>{r:.6f} {g:.6f} {b:.6f} {a:.6f}</diffuse>" in text
