"""Robot skills: the only place that sequences arm motions.

Everything here is blocking; the server runs it in a worker thread.
All moves go through RoArm.move_to(), which enforces real speed limits:
    travel_mm_s  in open space
    near_mm_s    for the last centimetres near the screen, chocolate and cup
    jog_mm_s     for calibration jogging
"""
from __future__ import annotations

import time
from dataclasses import dataclass

from . import poses as P
from .arm import RoArm


@dataclass
class SkillConfig:
    # speeds (mm/s, mm/s^2)
    travel_mm_s: float = 50.0     # at 100 % on the operator page
    near_mm_s: float = 15.0
    jog_mm_s: float = 10.0
    accel_mm_s2: float = 100.0
    # tapping
    hover_mm: float = 25.0         # height above the screen where the slow part starts
    press_mm: float = 3.0          # how far below the taught touch height to press
    dwell_s: float = 0.12
    point_mm: float = 12.0         # "point" mode: stop this high above the screen
    point_dwell_s: float = 0.8
    # chocolate
    pick_hover_mm: float = 50.0
    cup_hover_mm: float = 40.0
    gripper_open: float = 2.2      # firmware rad, tune on the operator page
    gripper_grip: float = 2.9
    gripper_spd: float = 300.0     # servo steps/s (4096 = one turn)
    gripper_acc: float = 10.0
    # old firmware-speed settings: still accepted in config.yaml, no longer used
    speed: float | None = None
    slow_speed: float | None = None


class Skills:
    def __init__(self, arm: RoArm, poses: dict, cfg: SkillConfig):
        self.arm, self.poses, self.cfg = arm, poses, cfg

    # -- motion helpers ------------------------------------------------------
    def _travel(self, x, y, z, **kw):
        self.arm.move_to(x, y, z, speed_mm_s=self.cfg.travel_mm_s, accel_mm_s2=self.cfg.accel_mm_s2, **kw)

    def _near(self, x, y, z, **kw):
        self.arm.move_to(x, y, z, speed_mm_s=self.cfg.near_mm_s, accel_mm_s2=self.cfg.accel_mm_s2, **kw)

    def _grip(self, rad):
        self.arm.gripper(rad, spd=self.cfg.gripper_spd, acc=self.cfg.gripper_acc)

    # -- game ---------------------------------------------------------------
    def tap(self, cell: int, extra_press: float = 0.0, touch: bool = True) -> None:
        """Touch the cell with the stylus, or (touch=False) just point at it."""
        x, y, z = P.cell_xyz(self.poses, cell)
        c = self.cfg
        self._travel(x, y, z + c.hover_mm)
        if touch:
            depth = c.press_mm + extra_press
            self._near(x, y, z - depth, tol=depth + 6)   # the screen stops the stylus early
            time.sleep(c.dwell_s)
        else:
            self._near(x, y, z + c.point_mm)
            time.sleep(c.point_dwell_s)
        self._near(x, y, z + c.hover_mm)

    def rest(self) -> None:
        """Park clear of the board so students can see and reach it."""
        self._travel(*self.poses["home"])

    def give(self, slot: int) -> None:
        c = self.cfg
        sx, sy, sz = self.poses["tray"][slot]
        cx, cy, cz = self.poses["cup"]
        self._travel(sx, sy, sz + c.pick_hover_mm)
        self._grip(c.gripper_open)
        self._near(sx, sy, sz)
        self._grip(c.gripper_grip)
        self._near(sx, sy, sz + c.pick_hover_mm)
        self._travel(cx, cy, cz + c.cup_hover_mm)
        self._near(cx, cy, cz)
        self._grip(c.gripper_open)
        self._near(cx, cy, cz + c.cup_hover_mm)
        self._grip(c.gripper_grip)
        self.rest()

    def wave(self) -> None:
        x, y, z = self.poses["home"]
        for dy in (40, -40, 40, 0):
            self._travel(x, y + dy, z - 20)   # a little below home, so it stays within reach
        self.rest()

    # -- calibration --------------------------------------------------------
    def goto(self, xyz) -> None:
        self._near(*xyz)

    def jog(self, axis: str, mm: float) -> list[float]:
        p = self.arm.pose()
        xyz = [p["x"], p["y"], p["z"]]
        xyz["xyz".index(axis)] += mm
        # jog steps are tiny, so they always use the jog speed (not the % setting)
        self.arm.move_to(*xyz, speed_mm_s=self.cfg.jog_mm_s, accel_mm_s2=self.cfg.accel_mm_s2, scaled=False)
        return xyz
