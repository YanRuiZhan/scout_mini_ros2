"""navigation.launch.py writes the requested AMCL start pose into the Nav2 params."""

import importlib.util
from pathlib import Path

import pytest
import yaml

pytest.importorskip("launch_ros")
ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("navigation_launch", ROOT / "launch/navigation.launch.py")
nav = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(nav)
PARAMS = ROOT / "config/nav2_params.yaml"


def amcl(path):
    return yaml.safe_load(Path(path).read_text())["amcl"]["ros__parameters"]


@pytest.mark.parametrize("text, expected", [
    ("0 0 0", (0.0, 0.0, 0.0)),
    ("1.5, -2 1.5708", (1.5, -2.0, 1.5708)),
    ("none", None),
    ("", None),
])
def test_parse_initial_pose(text, expected):
    assert nav.parse_initial_pose(text) == expected


@pytest.mark.parametrize("text", ["1 2", "a b c", "1 2 nan"])
def test_parse_initial_pose_rejects_bad_input(text):
    with pytest.raises(RuntimeError, match="initial_pose"):
        nav.parse_initial_pose(text)


def test_rewritten_params_set_the_pose_and_keep_everything_else():
    path = nav.with_initial_pose(PARAMS, (1.0, -2.0, 0.5))
    try:
        assert amcl(path)["set_initial_pose"] is True
        assert amcl(path)["initial_pose"] == {"x": 1.0, "y": -2.0, "z": 0.0, "yaw": 0.5}
        original, rewritten = yaml.safe_load(PARAMS.read_text()), yaml.safe_load(Path(path).read_text())
        for params in (original, rewritten):
            del params["amcl"]
        assert rewritten == original
    finally:
        Path(path).unlink()


def test_none_leaves_the_pose_to_rviz():
    path = nav.with_initial_pose(PARAMS, None)
    try:
        assert amcl(path)["set_initial_pose"] is False
    finally:
        Path(path).unlink()
