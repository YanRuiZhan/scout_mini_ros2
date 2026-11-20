"""sim_patrol.launch.py helpers: the patrol command line and early path checks."""

import importlib.util
from pathlib import Path
import sys

import pytest

pytest.importorskip("launch_ros")
ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("sim_patrol_launch", ROOT / "launch/sim_patrol.launch.py")
launch = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(launch)


def test_online_patrol_uses_poi():
    cmd = launch.patrol_command("/lib/poi_patrol.py", "/m.yaml", poi="/p.yaml", startup_timeout=120)
    assert cmd == [sys.executable, "/lib/poi_patrol.py", "--poi", "/p.yaml", "--startup-timeout", "120"]


def test_offline_plan_passes_map_for_staleness_check_and_extra_args():
    cmd = launch.patrol_command("/s.py", "/m.yaml", poi="/p.yaml", plan="/plan.yaml",
                                extra="--only PG-W01 'AP W02' --order greedy")
    assert cmd[2:6] == ["--plan", "/plan.yaml", "--map", "/m.yaml"]
    assert "--poi" not in cmd
    assert cmd[-5:] == ["--only", "PG-W01", "AP W02", "--order", "greedy"]


def test_missing_inputs_fail_early(tmp_path):
    with pytest.raises(RuntimeError, match="poi"):
        launch.patrol_command("/s.py", "/m.yaml")
    with pytest.raises(RuntimeError, match="不存在"):
        launch.existing_path(str(tmp_path / "nope.yaml"), "地图 map")
    (tmp_path / "m.yaml").write_text("x")
    assert launch.existing_path(str(tmp_path / "m.yaml"), "地图") == str((tmp_path / "m.yaml").resolve())
