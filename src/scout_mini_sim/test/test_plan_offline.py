"""Offline viewpoint planning on synthetic maps: constraints, occlusion, diversity, output."""
import argparse
import importlib.util
import math
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
SPEC = importlib.util.spec_from_file_location("plan_offline", SCRIPTS / "plan_offline.py")
po = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(po)

RES = 0.05


def write_map(tmp_path, boxes, size=(12.0, 10.0)):
    """12 x 10 m map with a 1-cell border wall plus the given occupied boxes (x0, y0, x1, y1)."""
    w, h = int(size[0] / RES), int(size[1] / RES)
    img = np.full((h, w), 254, dtype=np.uint8)            # free
    img[0, :] = img[-1, :] = img[:, 0] = img[:, -1] = 0
    for x0, y0, x1, y1 in boxes:
        c0, c1 = int(x0 / RES), int(x1 / RES)
        r0, r1 = h - int(y1 / RES), h - int(y0 / RES)     # PGM rows run top to bottom
        img[r0:r1, c0:c1] = 0
    (tmp_path / "m.pgm").write_bytes(f"P5\n{w} {h}\n255\n".encode() + img.tobytes())
    (tmp_path / "m.yaml").write_text("image: m.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\n"
                                     "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
    return tmp_path / "m.yaml"


def write_poi(tmp_path, z=None, size=None):
    poi = {"id": "D1", "type": "pressure_gauge", "target": {"x": 1.0, "y": 5.0},
           "normal": {"x": 1.0, "y": 0.0}}
    if z is not None:
        poi.update(z=z, size=size)
    (tmp_path / "poi.yaml").write_text(yaml.safe_dump({"frame_id": "map", "pois": [poi]}))
    return tmp_path / "poi.yaml"


PILLAR = (0.5, 4.5, 1.0, 5.5)          # the device hangs on its +x face at (1.0, 5.0)
BLOCKER = (2.6, 4.75, 3.0, 5.25)       # a box straight in front of the device


def params(**over):
    p = po.build_parser().parse_args(["--poi", "x", "--map", "x"])
    for k, v in over.items():
        setattr(p, k, v)
    return p


def setup(tmp_path, boxes=(PILLAR, BLOCKER), z=0.6, size=None):
    grid = po.Grid(write_map(tmp_path, boxes))
    poi = {"id": "D1", "type": "pressure_gauge", "target": {"x": 1.0, "y": 5.0},
           "normal": {"x": 1.0, "y": 0.0}}
    geo = {"z": z, "w": 0.3, "h": 0.3} if size is None else dict(z=z, **size)
    return grid, poi, geo


def test_clearance_and_cells(tmp_path):
    grid, _, _ = setup(tmp_path)
    # 1 m from the pillar face, 0.4 m from the blocker's near face
    assert grid.clearance[grid.cell(2.0, 5.0)] == pytest.approx(0.6, abs=0.06)
    assert grid.clearance[grid.cell(6.0, 8.0)] == pytest.approx(2.0)     # capped
    assert grid.cell(-1, 5) is None and grid.occ[grid.cell(0.7, 5.0)]


def test_blocked_view_is_rejected_and_others_survive(tmp_path):
    grid, poi, geo = setup(tmp_path)
    p = params()
    straight = {"x": 4.0, "y": 5.0, "yaw": math.pi, "r": 3.0, "phi_deg": 0.0}
    assert po.evaluate(straight, poi, geo, grid, None, None, p) == (None, "occluded")
    chosen, rejected = po.plan_device(poi, geo, grid, None, None, p)
    assert rejected["occluded"] > 0 and len(chosen) == 3
    for c in chosen:
        cell = grid.cell(c["x"], c["y"])
        assert grid.known_free[cell] and grid.clearance[cell] >= 0.6 - 1e-9
        assert c["visibility"] >= 0.75
    # alternatives come from different directions
    phis = sorted(c["phi_deg"] for c in chosen)
    assert all(b - a >= 30 for a, b in zip(phis, phis[1:]))
    # without the blocker the best view looks almost straight at the face
    best = po.plan_device(poi, geo, po.Grid(write_map(tmp_path, [PILLAR])), None, None, p)[0][0]
    assert abs(best["phi_deg"]) <= 10


def test_high_device_needs_distance_for_vertical_fov(tmp_path):
    grid, poi, _ = setup(tmp_path, boxes=[PILLAR])
    p = params()
    # face 1.3..1.7 m high, camera at 0.4 m: the top needs ~4 m, more than r_max allows
    chosen, rejected = po.plan_device(poi, {"z": 1.5, "w": 0.3, "h": 0.4}, grid, None, None, p)
    assert not chosen and rejected["fov"] > 0
    # with a farther r_max the planner backs off until the whole face fits
    chosen, _ = po.plan_device(poi, {"z": 1.5, "w": 0.3, "h": 0.4}, grid, None, None,
                               params(r_max=6.0))
    half_v = po.vertical_fov() / 2 - p.fov_margin
    for c in chosen:
        assert math.atan2(1.7 - 0.4, c["r"] - po.CAMERA["forward"]) <= half_v + 1e-9


def test_voxels_see_over_low_boxes(tmp_path):
    # A 0.3 m tall box between camera (0.4 m) and a target 1.0 m high: the 3D check sees
    # over it, the 2D grid (which treats every obstacle as full height) does not.
    xs, ys, zs = np.meshgrid(np.arange(2.6, 3.0, 0.05), np.arange(4.8, 5.2, 0.05),
                             np.arange(0.1, 0.3, 0.05))
    box = np.column_stack([xs.ravel(), ys.ravel(), zs.ravel()])
    floor = np.column_stack([np.random.default_rng(0).uniform(0, 12, (2000, 2)),
                             np.zeros(2000)])
    vox = po.Voxels(np.vstack([box, floor]))
    assert vox.ground_z == pytest.approx(0.0, abs=1e-6)
    cam, target = (4.0, 5.0, 0.4), (1.0, 5.0, 1.0)
    assert not vox.segment_blocked(cam, target, 0.25)
    assert vox.segment_blocked((4.0, 5.0, 0.2), (1.0, 5.0, 0.2), 0.25)   # low line of sight
    grid, _, _ = setup(tmp_path)
    assert grid.segment_blocked(cam[:2], target[:2], 0.25)


def test_pcd_reader_ascii_and_binary(tmp_path):
    pts = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype="<f4")
    head = "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nPOINTS 2\n"
    (tmp_path / "a.pcd").write_text(head + "DATA ascii\n1 2 3\n4 5 6\n")
    (tmp_path / "b.pcd").write_bytes((head + "DATA binary\n").encode() + pts.tobytes())
    for name in ("a.pcd", "b.pcd"):
        assert po.read_pcd_xyz(tmp_path / name).tolist() == pts.tolist()


def test_geometry_from_world_mesh(tmp_path):
    (tmp_path / "w.sdf").write_text(
        "<sdf><world><include><name>poi_d_1</name><pose>1 2 0.1 0 0 0</pose></include></world></sdf>")
    mesh = tmp_path / "models/poi_d_1/meshes"
    mesh.mkdir(parents=True)
    mesh.joinpath("face.obj").write_text(
        "v -0.2 -0.05 1.0\nv 0.2 -0.05 1.0\nv 0.2 -0.05 1.6\nv -0.2 -0.05 1.6\nf 1 2 3 4\n")
    geo = po.geometry_from_world(tmp_path / "w.sdf", ["D-1", "missing"])
    assert geo == {"D-1": {"z": 1.4, "w": 0.4, "h": 0.6}}
    poi = {"id": "X", "type": "access_panel"}
    assert po.device_geometry(poi, {}) == (po.TYPE_DEFAULTS["access_panel"], "default")
    assert po.device_geometry(dict(poi, z=0.9, size={"w": 0.2, "h": 0.2}), {})[1] == "poi"


def test_main_writes_plan_with_matrix_and_hashes(tmp_path, capsys):
    map_yaml = write_map(tmp_path, [PILLAR, BLOCKER])
    poi = write_poi(tmp_path, z=0.6, size={"w": 0.3, "h": 0.3})
    out = tmp_path / "plan.yaml"
    assert po.main(["--poi", str(poi), "--map", str(map_yaml), "--start", "10", "5",
                    "--out", str(out), "--grid-step", "4"]) == 0
    plan = yaml.safe_load(out.read_text())
    n = len(plan["viewpoints"])
    assert n == 3 and plan["devices"][0]["viewpoints"] == [0, 1, 2]
    assert plan["devices"][0]["geometry"]["source"] == "poi"
    m = np.array(plan["matrix"])
    assert m.shape == (n, n) and np.allclose(np.diag(m), 0) and np.allclose(m, m.T, rtol=0.05)
    assert all(math.isfinite(d) and d > 5 for d in plan["start"]["row"])
    assert plan["sources"]["map"]["sha1"] == po.file_sha1(*po.map_files(map_yaml))
    assert [v["rank"] for v in plan["viewpoints"]] == [1, 2, 3]
    assert "已写入" in capsys.readouterr().out


def test_unreachable_start_is_reported(tmp_path, capsys):
    map_yaml = write_map(tmp_path, [PILLAR])
    poi = write_poi(tmp_path)
    # a start inside the pillar snaps to the nearest free cell, so planning proceeds;
    # a start outside the map is an error
    assert po.main(["--poi", str(poi), "--map", str(map_yaml), "--start", "0.7", "5.0",
                    "--out", str(tmp_path / "p.yaml")]) == 0
    assert po.main(["--poi", str(poi), "--map", str(map_yaml), "--start", "50", "50",
                    "--out", str(tmp_path / "p.yaml")]) == 1
    assert "起点" in capsys.readouterr().err
