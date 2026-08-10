from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from inject_iris_sensors import (  # noqa: E402
    DEFAULT_OF_MODE,
    MOCKUP_PLUGIN_SO,
    inject,
    resolve_of_mode,
)

BARE_IRIS = """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='iris_1'>
    <link name='base_link'/>
  </model>
</sdf>
"""

LIDAR_ONLY_IRIS = """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='iris_1'>
    <link name='base_link'/>
    <include>
      <uri>model://lidar</uri>
      <name>lidar</name>
    </include>
    <joint name="lidar_joint" type="fixed">
      <parent>base_link</parent>
      <child>lidar::link</child>
    </joint>
  </model>
</sdf>
"""


def _write(tmp_path: Path, text: str) -> Path:
    sdf = tmp_path / "iris_1.sdf"
    sdf.write_text(text, encoding="utf-8")
    return sdf


def test_default_mode_is_mockup_plus_lidar(tmp_path: Path):
    sdf = _write(tmp_path, BARE_IRIS)
    inject(sdf)
    text = sdf.read_text(encoding="utf-8")
    assert DEFAULT_OF_MODE == "mockup"
    assert MOCKUP_PLUGIN_SO in text
    assert 'name="lidar_joint"' in text
    assert "<min_distance>0.02</min_distance>" in text
    # Headless default must not pull in the render-dependent camera plugin.
    assert 'model://px4flow' not in text


def test_camera_mode_adds_px4flow_and_lidar(tmp_path: Path):
    sdf = _write(tmp_path, BARE_IRIS)
    inject(sdf, of_mode="camera")
    text = sdf.read_text(encoding="utf-8")
    assert 'model://px4flow' in text
    assert 'name="px4flow_joint"' in text
    assert 'name="lidar_joint"' in text
    assert MOCKUP_PLUGIN_SO not in text


def test_both_mode_adds_every_source(tmp_path: Path):
    sdf = _write(tmp_path, BARE_IRIS)
    inject(sdf, of_mode="both")
    text = sdf.read_text(encoding="utf-8")
    assert 'model://px4flow' in text
    assert 'name="lidar_joint"' in text
    assert MOCKUP_PLUGIN_SO in text


@pytest.mark.parametrize("mode", ["mockup", "camera", "both"])
def test_inject_is_idempotent(tmp_path: Path, mode: str):
    sdf = _write(tmp_path, BARE_IRIS)
    inject(sdf, of_mode=mode)
    once = sdf.read_text(encoding="utf-8")
    inject(sdf, of_mode=mode)
    assert sdf.read_text(encoding="utf-8") == once


@pytest.mark.parametrize("mode", ["mockup", "camera"])
def test_lidar_only_sdf_is_upgraded_without_duplicate_lidar(tmp_path: Path, mode: str):
    sdf = _write(tmp_path, LIDAR_ONLY_IRIS)
    inject(sdf, of_mode=mode)
    text = sdf.read_text(encoding="utf-8")
    # Legacy model://lidar counts as already present — do not double-inject.
    assert text.count("model://lidar") == 1
    assert text.count('name="lidar_joint"') == 1
    if mode == "mockup":
        assert MOCKUP_PLUGIN_SO in text
    else:
        assert 'model://px4flow' in text


def test_blocks_land_inside_the_model_element(tmp_path: Path):
    sdf = _write(tmp_path, BARE_IRIS)
    inject(sdf, of_mode="both")
    text = sdf.read_text(encoding="utf-8")
    assert text.rfind(MOCKUP_PLUGIN_SO) < text.rfind("</model>")
    assert text.rfind("model://px4flow") < text.rfind("</model>")


def test_env_override(monkeypatch):
    monkeypatch.setenv("CATSWARM_OF_MODE", "camera")
    assert resolve_of_mode() == "camera"
    monkeypatch.delenv("CATSWARM_OF_MODE")
    assert resolve_of_mode() == DEFAULT_OF_MODE


def test_invalid_mode_rejected():
    with pytest.raises(SystemExit):
        resolve_of_mode("bogus")
