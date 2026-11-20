"""Tour planner: optimality, fallbacks and grid-map distances, without ROS."""

import importlib.util
import itertools
import math
from pathlib import Path
import random

import pytest

SPEC = importlib.util.spec_from_file_location(
    "tour_planner", Path(__file__).resolve().parents[1] / "scripts/tour_planner.py")
tp = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(tp)


def brute_force(D, return_home):
    nodes = range(1, len(D))
    return min(tp.tour_length(list(p), D, return_home) for p in itertools.permutations(nodes))


@pytest.mark.parametrize("return_home", [True, False])
@pytest.mark.parametrize("seed", range(6))
def test_exact_matches_brute_force(seed, return_home):
    rng = random.Random(seed)
    pts = [(rng.uniform(-10, 10), rng.uniform(-10, 10)) for _ in range(8)]
    # asymmetric costs too: the planner may report slightly different lengths each way
    D = [[0 if i == j else tp.euclidean(a, b) * (1 + 0.2 * rng.random()) for j, b in enumerate(pts)]
         for i, a in enumerate(pts)]
    plan = tp.plan_tour(pts[0], pts[1:], method="exact", return_home=return_home, matrix=D)
    assert plan.total == pytest.approx(brute_force(D, return_home))
    assert sorted(plan.order) == list(range(7))


def test_greedy_detour_is_fixed_by_exact_and_two_opt():
    # Start in the middle of a corridor; greedy runs to the near end first and then has to
    # cross back over the start, exact and 2-opt visit one side and come back once.
    start, wps = (0, 0), [(1, 0), (-2, 0), (3, 0), (-4, 0), (5, 0)]
    greedy = tp.plan_tour(start, wps, method="greedy")
    exact = tp.plan_tour(start, wps, method="exact")
    two_opt = tp.plan_tour(start, wps, method="two_opt")
    assert exact.total == pytest.approx(18.0)     # out to 5, back to -4, home
    assert two_opt.total <= greedy.total
    assert exact.total <= two_opt.total + 1e-9
    assert len(exact.legs) == len(wps) + 1         # includes the leg back to the start


@pytest.mark.parametrize("method", ["exact", "two_opt"])
@pytest.mark.parametrize("seed", range(5))
def test_closed_tour_direction_is_fixed(seed, method):
    # A closed tour has the same length both ways; the planner must always pick the
    # direction that starts at the end nearer the start, whatever the input order.
    rng = random.Random(seed)
    start = (0.0, 0.0)
    wps = [(rng.uniform(-20, 20), rng.uniform(-20, 20)) for _ in range(9)]
    plan = tp.plan_tour(start, wps, method=method)
    assert plan.legs[0] <= plan.legs[-1] + 1e-9
    visited = [wps[k] for k in plan.order]
    for shuffle_seed in range(3):
        shuffled = wps[:]
        random.Random(shuffle_seed).shuffle(shuffled)
        again = tp.plan_tour(start, shuffled, method=method)
        assert [shuffled[k] for k in again.order] == visited


def test_orient_tour_prefers_shorter_direction_and_keeps_open_tours():
    # asymmetric costs: 0 -> 1 -> 2 -> 0 costs 3, the reverse 0 -> 2 -> 1 -> 0 costs 30
    D = [[0, 1, 10], [10, 0, 1], [1, 10, 0]]
    assert tp.orient_tour([2, 1], D, return_home=True) == [1, 2]
    assert tp.orient_tour([2, 1], D, return_home=False) == [2, 1]
    # equal lengths and equal first legs: lower index first
    S = [[0, 5, 5], [5, 0, 3], [5, 3, 0]]
    assert tp.orient_tour([2, 1], S) == [1, 2]


def test_greedy_baseline_is_not_reoriented():
    # greedy goes to the nearest point first even if the far end would come first otherwise
    start, wps = (0, 0), [(1, 0), (2, 0), (2, 5), (0, 5)]
    plan = tp.plan_tour(start, wps, method="greedy")
    assert plan.order[0] == 0


def test_open_tour_has_no_return_leg():
    plan = tp.plan_tour((0, 0), [(1, 0), (2, 0)], method="exact", return_home=False)
    assert plan.order == [0, 1] and plan.legs == [1.0, 1.0] and plan.total == 2.0


def test_auto_switches_to_two_opt_for_many_points():
    rng = random.Random(1)
    wps = [(rng.uniform(0, 50), rng.uniform(0, 50)) for _ in range(20)]
    assert tp.plan_tour((0, 0), wps[:5], method="auto").method == "exact"
    plan = tp.plan_tour((0, 0), wps, method="auto", exact_limit=13)
    assert plan.method == "two_opt" and sorted(plan.order) == list(range(20))


def test_unreachable_points_are_reported_and_skipped():
    def dist(a, b):
        return math.inf if 99 in (a[0], b[0]) else tp.euclidean(a, b)
    plan = tp.plan_tour((0, 0), [(1, 0), (99, 0), (2, 0)], distance=dist, method="exact")
    assert plan.unreachable == [1]
    assert sorted(plan.order) == [0, 2]
    assert math.isfinite(plan.total)


def test_failed_pair_falls_back_to_going_via_the_start():
    D = [[0, 1, 1], [1, 0, math.inf], [1, math.inf, 0]]
    plan = tp.plan_tour((0, 0), [None, None], method="exact", matrix=D)
    assert plan.total == pytest.approx(1 + 2 + 1)


def test_symmetric_matrix_halves_the_calls():
    calls = []

    def dist(a, b):
        calls.append((a, b))
        return tp.euclidean(a, b)
    tp.distance_matrix([(0, 0), (1, 0), (0, 1), (1, 1)], dist, symmetric=True)
    assert len(calls) == 6


def test_grid_map_distance_goes_around_a_wall(tmp_path):
    # 10 x 10 m map at 0.1 m, a wall at x = 5 m from y = 0 to 8 m: (2,2) -> (8,2) must go
    # around its top end, so the path is much longer than the 6 m straight line.
    w = h = 100
    pixels = bytearray([254] * (w * h))
    for row in range(h):                      # PGM rows run top to bottom
        y = (h - 1 - row) * 0.1
        if y <= 8.0:
            for col in (49, 50, 51):
                pixels[row * w + col] = 0
    (tmp_path / "m.pgm").write_bytes(b"P5\n100 100\n255\n" + bytes(pixels))
    (tmp_path / "m.yaml").write_text(
        "image: m.pgm\nresolution: 0.1\norigin: [0.0, 0.0, 0.0]\nnegate: 0\n"
        "occupied_thresh: 0.65\nfree_thresh: 0.25\n")
    grid = tp.GridMap(tmp_path / "m.yaml", robot_radius=0.3, step=1)
    D = grid.distances([(2.0, 2.0), (8.0, 2.0), (2.0, 4.0)])
    # two diagonals over the wall's end (about 2 x 7.1 m) instead of the 6 m straight line;
    # an 8-connected grid overestimates diagonals by up to ~8 %
    assert 14.0 < D[0][1] < 14.2 * 1.1
    assert D[0][2] == pytest.approx(2.0, abs=0.15)
    assert D[0][1] == pytest.approx(D[1][0], rel=1e-6)


def test_read_pgm_ascii(tmp_path):
    (tmp_path / "a.pgm").write_text("P2\n# comment\n2 1\n255\n0 254\n")
    assert tp.read_pgm(tmp_path / "a.pgm") == (2, 1, [0, 254])


def corridor_map(tmp_path):
    """20 x 6 m corridor at 0.1 m: walls along y = 0 and y = 6, plus a crate near the lower wall."""
    w, h = 200, 60
    pixels = bytearray([254] * (w * h))
    for col in range(w):
        for row in (0, h - 1):
            pixels[row * w + col] = 0
    for row in range(h - 15, h - 1):            # crate x 9..11 m, y 0..1.5 m
        for col in range(90, 110):
            pixels[row * w + col] = 0
    (tmp_path / "c.pgm").write_bytes(b"P5\n200 60\n255\n" + bytes(pixels))
    (tmp_path / "c.yaml").write_text("image: c.pgm\nresolution: 0.1\norigin: [0.0, 0.0, 0.0]\n"
                                     "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
    return tmp_path / "c.yaml"


COSTMAP = {"inflation_radius": 1.0, "cost_scaling_factor": 2.0, "travel_multiplier": 2.0,
           "inscribed_radius": 0.31}


def test_cost_aware_path_keeps_away_from_obstacles(tmp_path):
    # Two points 0.6 m above the lower wall: the plain shortest path skims the crate,
    # the Nav2-like cost model swings out into the corridor, so it is longer.
    path = corridor_map(tmp_path)
    plain = tp.GridMap(path, robot_radius=0.31, step=1)
    costed = tp.GridMap(path, robot_radius=0.31, step=1, costmap=COSTMAP)
    a, b = (2.0, 2.0), (18.0, 2.0)
    d_plain = plain.distances([a, b])[0][1]
    d_cost = costed.distances([a, b], method="astar")[0][1]
    assert d_plain == pytest.approx(16.0)       # straight line, 0.5 m above the crate
    assert d_cost > d_plain + 0.05               # detours into the middle of the corridor
    # A* and Dijkstra find the same optimum under the same costs
    assert costed.distances([a, b], method="dijkstra")[0][1] == pytest.approx(d_cost, rel=1e-6)
    # far from every obstacle the multiplier is exactly 1
    assert costed.mult.get(costed.cell(10.0, 3.6), 1.0) == pytest.approx(1.0)
    assert costed.mult[costed.cell(10.0, 2.0)] > 1.0


def test_astar_matrix_is_symmetric_and_reports_progress(tmp_path):
    grid = tp.GridMap(corridor_map(tmp_path), robot_radius=0.31, step=2, costmap=COSTMAP)
    calls = []
    D = grid.distances([(2, 3), (10, 4), (18, 3)], method="astar",
                       progress=lambda k, n: calls.append((k, n)))
    assert calls[-1] == (3, 3)
    assert all(D[i][j] == D[j][i] for i in range(3) for j in range(3))
    assert D[0][2] == pytest.approx(D[0][1] + D[1][2], rel=0.1)
