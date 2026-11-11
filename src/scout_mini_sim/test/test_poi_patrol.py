"""Pure-data checks for the warehouse inspection patrol script."""

import importlib.util
from pathlib import Path

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/poi_patrol.py"
SPEC = importlib.util.spec_from_file_location("poi_patrol", SCRIPT)
patrol = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(patrol)


def test_warehouse_pois_match_scene_and_face_orientation():
    config = SCRIPT.parents[1] / "config/poi_warehouse_inspection.yaml"
    pois = patrol.load_pois(config)

    assert len(pois) == 10
    pg_w01 = next(poi for poi in pois if poi["id"] == "PG-W01")
    x, y, yaw = patrol.view_pose(pg_w01, 1.5)
    assert (x, y) == pytest.approx((7.65, -12.498))
    assert yaw == pytest.approx(-pytest.approx(0) if False else 0.0)


def test_load_pois_rejects_zero_normal(tmp_path):
    config = tmp_path / "pois.yaml"
    config.write_text(
        "pois:\n"
        "  - id: test\n"
        "    type: gauge\n"
        "    target: {x: 0, y: 0}\n"
        "    normal: {x: 0, y: 0}\n",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="normal 不能为零向量"):
        patrol.load_pois(config)
