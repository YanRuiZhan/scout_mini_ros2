import importlib.util
from pathlib import Path

import numpy as np
import pytest

SPEC = importlib.util.spec_from_file_location(
    "pcd_to_grid", Path(__file__).parents[1] / "scripts" / "pcd_to_grid.py"
)
grid_tool = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(grid_tool)


def test_projection_priority_unknown_and_relative_height():
    xyz = np.array([[0, 0, -0.12], [0, 0, 0.3], [1, 1, -0.12], [0.5, 0.5, 3.0]])
    grid, origin = grid_tool.project(xyz, 0.1, -0.12, 0.08, 0.15, 1.8, 1, 0)
    assert np.allclose(origin, [0, 0])
    assert grid[0, 0] == 0
    assert grid[10, 10] == 254
    assert grid[5, 5] == 205


def test_binary_fields_and_finite_filter(tmp_path):
    path = tmp_path / "test.pcd"
    header = ("FIELDS intensity x y z\nSIZE 4 4 4 4\nTYPE F F F F\n"
              "COUNT 1 1 1 1\nPOINTS 2\nDATA binary\n")
    path.write_bytes(header.encode() + np.array([[7, 1, 2, 3], [0, np.nan, 0, 0]], dtype="<f4").tobytes())
    assert np.array_equal(grid_tool.read_xyz(path), [[1, 2, 3]])


def test_image_flip_and_no_overwrite(tmp_path):
    prefix = tmp_path / "map"
    grid = np.array([[254, 205], [0, 254]], dtype=np.uint8)
    image, metadata = grid_tool.save_map(prefix, grid, [0, 0], 0.05)
    assert image.read_bytes().split(b"\n", 3)[3] == bytes([0, 254, 254, 205])
    assert "resolution: 0.05" in metadata.read_text()
    with pytest.raises(ValueError, match="already exists"):
        grid_tool.save_map(prefix, grid, [0, 0], 0.05)


def test_invalid_height_band():
    with pytest.raises(ValueError, match="ground_tolerance"):
        grid_tool.project(np.array([[0, 0, 0]]), 0.05, 0, 0.2, 0.1, 1.0)


def test_sensor_height_includes_lower_obstacles():
    xyz = np.array([
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.5],
        [1.0, 0.0, 0.2],
        [2.0, 0.0, 0.0],
        [1.5, 0.0, 0.8],
    ])
    grid, _ = grid_tool.project(
        xyz, 0.1, 0.0, 0.05, 0.15, 1.8, 1, 0,
        sensor_height=0.5, sensor_tolerance=0.05,
    )
    assert grid[0, 0] == 0
    assert grid[0, 10] == 0
    assert grid[0, 15] == 205
