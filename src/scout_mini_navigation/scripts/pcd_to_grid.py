#!/usr/bin/python3
"""Project an upright PCD into a floor-evidence-based Nav2 occupancy map."""

import argparse
from pathlib import Path

import numpy as np


def read_xyz(path):
    """Read uncompressed binary or ASCII PCD, respecting field sizes/counts."""
    header = {}
    with Path(path).open("rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                raise ValueError("Missing PCD DATA header")
            text = line.decode("ascii").strip()
            if not text or text.startswith("#"):
                continue
            key, *values = text.split()
            header[key] = values
            if key == "DATA":
                break
        fields = header["FIELDS"]
        sizes = list(map(int, header["SIZE"]))
        types = header["TYPE"]
        counts = list(map(int, header.get("COUNT", ["1"] * len(fields))))
        if not (len(fields) == len(sizes) == len(types) == len(counts)):
            raise ValueError("Inconsistent PCD field metadata")
        for axis in ("x", "y", "z"):
            if axis not in fields or counts[fields.index(axis)] != 1:
                raise ValueError("PCD requires scalar x/y/z fields")
        n = int(header["POINTS"][0])
        mode = header["DATA"][0]
        if mode == "binary":
            dtype = np.dtype([
                (name, "<" + {"F": "f", "I": "i", "U": "u"}[kind] + str(size), (count,))
                for name, size, kind, count in zip(fields, sizes, types, counts)
            ])
            data = np.frombuffer(stream.read(), dtype=dtype)
            if len(data) != n:
                raise ValueError("PCD payload size does not match POINTS")
            xyz = np.column_stack([data[axis][:, 0] for axis in ("x", "y", "z")])
        elif mode == "ascii":
            data = np.loadtxt(stream, ndmin=2)
            if data.shape != (n, sum(counts)):
                raise ValueError("PCD ASCII shape does not match header")
            offsets = np.cumsum([0] + counts[:-1])
            xyz = data[:, [offsets[fields.index(axis)] for axis in ("x", "y", "z")]]
        else:
            raise ValueError("binary_compressed is not supported; export binary/ASCII PCD first")
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if not len(xyz):
        raise ValueError("PCD contains no finite points")
    return xyz


def project(xyz, resolution, ground_z, ground_tolerance, min_height, max_height,
            min_points=2, padding=0.5):
    """Unknown=205, free=254, occupied=0; obstacle evidence wins."""
    values = [resolution, ground_z, ground_tolerance, min_height, max_height, padding]
    if not np.isfinite(values).all():
        raise ValueError("All projection parameters must be finite")
    if resolution <= 0 or ground_tolerance <= 0 or padding < 0 or min_points < 1:
        raise ValueError("Invalid resolution, tolerance, padding or point threshold")
    if not ground_tolerance < min_height < max_height:
        raise ValueError("Require ground_tolerance < min_height < max_height")
    relative_z = xyz[:, 2] - ground_z
    floor = np.abs(relative_z) <= ground_tolerance
    obstacle = (relative_z >= min_height) & (relative_z <= max_height)
    selected = floor | obstacle
    if not floor.any() or not obstacle.any():
        raise ValueError("No floor or obstacle evidence; check ground and height parameters")
    xy = xyz[selected, :2]
    origin = np.floor((xy.min(axis=0) - padding) / resolution) * resolution
    shape = np.floor((xy.max(axis=0) + padding - origin) / resolution).astype(int) + 1
    width, height = map(int, shape)
    if width * height > 25_000_000:
        raise ValueError("Map exceeds 25 million cells; crop cloud or increase resolution")
    indices = np.floor((xyz[:, :2] - origin) / resolution).astype(np.int64)

    def evidence(mask):
        cells = indices[mask]
        return np.bincount(cells[:, 1] * width + cells[:, 0], minlength=width * height)

    grid = np.full(width * height, 205, dtype=np.uint8)
    grid[evidence(floor) >= min_points] = 254
    grid[evidence(obstacle) >= min_points] = 0
    return grid.reshape(height, width), origin


def save_map(prefix, grid, origin, resolution):
    prefix = Path(prefix)
    image = prefix.with_suffix(".pgm")
    metadata = prefix.with_suffix(".yaml")
    # Do not silently replace an existing map or its metadata.
    if image.exists() or metadata.exists():
        raise ValueError(f"Output already exists: {image} or {metadata}; choose a new prefix")
    with image.open("wb") as stream:
        stream.write(f"P5\n{grid.shape[1]} {grid.shape[0]}\n255\n".encode("ascii"))
        stream.write(np.flipud(grid).tobytes())
    # YAML single-quoted strings escape a quote by doubling it.
    image_name = image.name.replace("'", "''")
    metadata.write_text(
        f"image: '{image_name}'\nmode: trinary\nresolution: {resolution}\n"
        f"origin: [{origin[0]:.8f}, {origin[1]:.8f}, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n",
        encoding="utf-8",
    )
    return image, metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path, help="Output prefix (without extension)")
    parser.add_argument("--ground-z", type=float, required=True, help="Floor Z in the PCD frame")
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--ground-tolerance", type=float, default=0.08)
    parser.add_argument("--min-height", type=float, default=0.15)
    parser.add_argument("--max-height", type=float, default=1.8)
    parser.add_argument("--min-points", type=int, default=2)
    parser.add_argument("--padding", type=float, default=0.5)
    args = parser.parse_args()
    try:
        xyz = read_xyz(args.input)
        grid, origin = project(
            xyz, args.resolution, args.ground_z, args.ground_tolerance,
            args.min_height, args.max_height, args.min_points, args.padding,
        )
        paths = save_map(args.output, grid, origin, args.resolution)
    except (ValueError, OSError, KeyError) as exc:
        parser.exit(1, f"Conversion failed: {exc}\n")
    print(f"Finite points: {len(xyz)}; map: {grid.shape[1]} x {grid.shape[0]}; origin: {origin}")
    for name, value in (("occupied", 0), ("free", 254), ("unknown", 205)):
        print(f"{name}: {np.count_nonzero(grid == value)} ({np.mean(grid == value):.1%})")
    print("Written:", *(str(path) for path in paths))
    print("Inspect before navigation: floor returns are only a free-space heuristic, not ray tracing.")


if __name__ == "__main__":
    main()
