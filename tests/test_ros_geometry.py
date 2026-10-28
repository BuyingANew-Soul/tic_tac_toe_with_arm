"""Tablet collision box geometry (no ROS needed)."""
import math

from ttt.ros_arm import tablet_box

CORNERS = {1: [-254.82, 300.97, -3.5], 3: [-203.53, 359.28, -8.45],
           7: [-177.94, 268.98, -50.73], 9: [-140.63, 317.26, -58.17]}


def _up(q):
    x, y, z, w = q
    return (2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y))


def test_box_follows_tilted_tablet_and_sits_below_touch_points():
    centre, size, q = tablet_box({"board_corners": CORNERS}, below_mm=30, margin_mm=45)
    assert math.isclose(sum(v * v for v in q), 1.0, rel_tol=1e-9)
    n = _up(q)
    assert 25 < math.degrees(math.acos(n[2])) < 38          # the tablet is tilted ~31 deg
    top = [centre[i] + n[i] * size[2] / 2 for i in range(3)]
    heights = [sum((CORNERS[k][i] - top[i]) * n[i] for i in range(3)) for k in (1, 3, 7, 9)]
    assert all(25 < h < 35 for h in heights)                 # every corner ~30 mm above the box


def test_no_box_without_all_corners():
    assert tablet_box({"board_corners": {1: [0, 0, 0]}}, 30, 45) is None
    assert tablet_box({}, 30, 45) is None
