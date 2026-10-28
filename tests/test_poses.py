import pytest

from ttt import poses as P


def test_interpolation_override_and_roundtrip(tmp_path):
    d = {}
    for n, xyz in [("corner_1", [300, 60, 10]), ("corner_3", [300, -60, 10]),
                   ("corner_7", [180, 60, 20]), ("corner_9", [180, -60, 20])]:
        P.set_pose(d, n, xyz)
    assert P.cell_xyz(d, 5) == pytest.approx([240, 0, 15])
    P.set_pose(d, "cell_5", [241, 1, 14])
    assert P.cell_xyz(d, 5) == [241, 1, 14]
    f = tmp_path / "p.yaml"
    P.save(d, f)
    assert P.cell_xyz(P.load(f), 9) == pytest.approx([180, -60, 20])
    assert "home" in P.missing(d)


def test_bad_names():
    with pytest.raises(ValueError):
        P.set_pose({}, "corner_2", [0, 0, 0])
    with pytest.raises(KeyError):
        P.cell_xyz({}, 1)
