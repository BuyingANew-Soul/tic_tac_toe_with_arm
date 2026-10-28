"""Taught poses (mm, rad, arm base frame from the RoArm firmware).

data/poses.yaml:
    board_corners: {1: [x,y,z], 3: [...], 7: [...], 9: [...]}  # stylus tip touching
    cells:  {5: [x,y,z]}          # optional per-cell overrides
    home:   [x,y,z]
    cup:    [x,y,z]               # prize drop point
    tray:   {1: [x,y,z], ...}     # gripper at grasp height over each chocolate

Only the 4 corner cells need teaching; the 5 others are interpolated.
"""
from __future__ import annotations

from pathlib import Path

import yaml


def load(path: Path) -> dict:
    path = Path(path)
    if not path.exists():
        return {}
    with open(path) as f:
        return yaml.safe_load(f) or {}


def save(data: dict, path: Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp")
    with open(tmp, "w") as f:
        yaml.safe_dump(data, f, sort_keys=True)
    tmp.replace(path)   # atomic: a crash never leaves a half-written file


def set_pose(data: dict, name: str, xyz) -> dict:
    """name: home, cup, corner_1/3/7/9, cell_N, tray_N."""
    xyz = [round(float(v), 1) for v in xyz]
    kind, _, num = name.partition("_")
    if kind in ("home", "cup") and not num:
        data[kind] = xyz
    elif kind == "corner" and num in ("1", "3", "7", "9"):
        data.setdefault("board_corners", {})[int(num)] = xyz
    elif kind == "cell" and num.isdigit() and 1 <= int(num) <= 9:
        data.setdefault("cells", {})[int(num)] = xyz
    elif kind == "tray" and num.isdigit():
        data.setdefault("tray", {})[int(num)] = xyz
    else:
        raise ValueError(f"bad pose name '{name}'")
    return data


def get_pose(data: dict, name: str) -> list[float]:
    kind, _, num = name.partition("_")
    if kind == "cell":
        return cell_xyz(data, int(num))
    if kind == "corner":
        return data["board_corners"][int(num)]
    if kind == "tray":
        return data["tray"][int(num)]
    return data[name]


def cell_xyz(data: dict, cell: int) -> list[float]:
    """Cell 1..9: explicit override, else bilinear from the four corners."""
    cells = data.get("cells") or {}
    if cell in cells:
        return cells[cell]
    c = data.get("board_corners") or {}
    missing = [k for k in (1, 3, 7, 9) if k not in c]
    if missing:
        raise KeyError(f"teach corner_{missing[0]} first")
    r, col = divmod(cell - 1, 3)
    u, v = col / 2.0, r / 2.0
    return [(1 - u) * (1 - v) * c[1][k] + u * (1 - v) * c[3][k]
            + (1 - u) * v * c[7][k] + u * v * c[9][k] for k in range(3)]


def missing(data: dict) -> list[str]:
    """What still has to be taught before the game can run."""
    out = [f"corner_{k}" for k in (1, 3, 7, 9) if k not in (data.get("board_corners") or {})]
    out += [k for k in ("home", "cup") if k not in data]
    if not data.get("tray"):
        out.append("tray_1..")
    return out


SIM_DEFAULTS = {
    "board_corners": {1: [300.0, 60.0, 10.0], 3: [300.0, -60.0, 10.0],
                      7: [180.0, 60.0, 15.0], 9: [180.0, -60.0, 15.0]},
    "home": [200.0, 0.0, 150.0],
    "cup": [220.0, 170.0, 60.0],
    "tray": {i: [160.0 + 30 * ((i - 1) % 3), -170.0 - 30 * ((i - 1) // 3), 30.0] for i in range(1, 7)},
}
