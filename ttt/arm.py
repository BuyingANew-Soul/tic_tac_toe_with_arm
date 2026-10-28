"""RoArm-M2 driver over USB serial (JSON line protocol).

The arm's ESP32 accepts one JSON object per line. Command IDs used here
follow Waveshare's RoArm-M2-S JSON protocol. If your firmware differs,
change the constants below; nothing else depends on the raw numbers.

Usage:
    arm = RoArm(sim=True).connect()      # no hardware needed
    arm = RoArm().connect()              # auto-detect USB port
    arm.move_xyz(235, 0, 150)
"""
from __future__ import annotations

import json
import math
import threading
import time
from dataclasses import dataclass

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # sim mode works without pyserial
    serial = None
    list_ports = None

# --- RoArm-M2-S JSON command IDs (verify against your firmware wiki) ---
CMD_HOME = 100          # {"T":100} move to initial pose
CMD_JOINTS_RAD = 102    # {"T":102,"base","shoulder","elbow","hand","spd","acc"}
CMD_XYZT = 104          # {"T":104,"x","y","z","t","spd"} interpolated by firmware
CMD_XYZT_DIRECT = 1041  # {"T":1041,"x","y","z","t"} no interpolation
CMD_FEEDBACK = 105      # {"T":105} -> reply {"T":1051,"x","y","z","b","s","e","t",...}
FEEDBACK_REPLY = 1051
CMD_GRIPPER_RAD = 106   # {"T":106,"cmd":rad,"spd","acc"}
CMD_TORQUE = 210        # {"T":210,"cmd":0|1}

# Gripper angles in radians. Tune on Day 1 with your chocolates.
GRIPPER_CLOSED = 3.14
GRIPPER_OPEN = 2.2

HOME_POSE = dict(x=235.0, y=0.0, z=234.0, t=GRIPPER_CLOSED)


class ArmError(Exception):
    pass


@dataclass
class Workspace:
    """Safety limits in mm (arm base frame). Every move is checked.

    - reach: the target must be within max_reach of the arm's base
    - floor: the target may not go below z_floor. The server sets the floor
      just under the lowest taught position, so the arm can't push into the
      table or the tablet. Before anything is taught, z_floor_default applies.
    """
    max_reach: float = 520.0         # RoArm-M2: ~0.5 m working radius
    z_floor_default: float = -250.0
    z_floor: float | None = None

    def check(self, x: float, y: float, z: float) -> None:
        reach = math.sqrt(x * x + y * y + z * z)
        floor = self.z_floor if self.z_floor is not None else self.z_floor_default
        bad = []
        if reach > self.max_reach:
            bad.append(f"{reach:.0f} mm from the base, more than the arm's reach ({self.max_reach:.0f} mm)")
        if z < floor:
            bad.append(f"z={z:.1f} is below the safety floor ({floor:.1f})")
        if bad:
            raise ArmError(f"Refused target ({x:.0f}, {y:.0f}, {z:.0f}): " + "; ".join(bad))


class BasePath:
    """Path between two points interpolated around the arm's base:
    base angle, distance from the base axis and height change together.
    The angle is interpolated WITHOUT wrapping, so it never crosses +-180 deg."""

    SAMPLES = 400

    def __init__(self, a, b):
        self.a, self.b = a, b
        self.th0, self.th1 = math.atan2(a[1], a[0]), math.atan2(b[1], b[0])
        self.r0, self.r1 = math.hypot(a[0], a[1]), math.hypot(b[0], b[1])
        # if either end sits on the base axis its angle is meaningless: use the other's
        if self.r0 < 1.0:
            self.th0 = self.th1
        if self.r1 < 1.0:
            self.th1 = self.th0
        self._u = [i / self.SAMPLES for i in range(self.SAMPLES + 1)]
        pts = [self._at(u) for u in self._u]
        self._s = [0.0]
        for p, q in zip(pts, pts[1:]):
            self._s.append(self._s[-1] + math.dist(p, q))
        self.length = self._s[-1]

    def _at(self, u: float):
        th = self.th0 + (self.th1 - self.th0) * u
        r = self.r0 + (self.r1 - self.r0) * u
        z = self.a[2] + (self.b[2] - self.a[2]) * u
        return (r * math.cos(th), r * math.sin(th), z)

    def point(self, s: float):
        """Point at distance s along the path."""
        if s <= 0:
            return self.a
        if s >= self.length:
            return self.b
        lo, hi = 0, len(self._s) - 1
        while hi - lo > 1:
            mid = (lo + hi) // 2
            if self._s[mid] <= s:
                lo = mid
            else:
                hi = mid
        seg = self._s[hi] - self._s[lo]
        u = self._u[lo] + (self._u[hi] - self._u[lo]) * ((s - self._s[lo]) / seg if seg else 0)
        return self._at(u)


class Trapezoid:
    """Distance along a path over time: accelerate, cruise at max speed, decelerate.
    Falls back to a triangle (no cruise) for short moves."""

    def __init__(self, dist: float, v_max: float, accel: float):
        self.d, self.a = dist, max(1e-6, accel)
        v_max = max(1e-6, v_max)
        t_acc = v_max / self.a
        d_acc = 0.5 * self.a * t_acc ** 2
        if 2 * d_acc >= dist:                    # never reaches full speed
            t_acc = math.sqrt(dist / self.a)
            v_max = self.a * t_acc
            d_acc = dist / 2
        self.v, self.t_acc, self.d_acc = v_max, t_acc, d_acc
        self.t_cruise = (dist - 2 * d_acc) / v_max
        self.duration = 2 * t_acc + self.t_cruise

    def position(self, t: float) -> float:
        if t <= 0:
            return 0.0
        if t >= self.duration:
            return self.d
        if t < self.t_acc:
            return 0.5 * self.a * t * t
        if t < self.t_acc + self.t_cruise:
            return self.d_acc + self.v * (t - self.t_acc)
        r = self.duration - t
        return self.d - 0.5 * self.a * r * r


def find_port() -> str | None:
    """Return the first USB serial port that looks like the RoArm's ESP32."""
    if list_ports is None:
        return None
    keys = ("cp210", "ch34", "ch910", "silicon labs", "esp32", "usb serial", "uart")
    for p in list_ports.comports():
        desc = f"{p.description} {p.manufacturer} {p.hwid}".lower()
        if any(k in desc for k in keys):
            return p.device
    return None


class RoArm:
    def __init__(self, port: str | None = None, baud: int = 115200, sim: bool = False,
                 workspace: Workspace | None = None, verbose: bool = False):
        self.port = port
        self.baud = baud
        self.sim = sim
        self.workspace = workspace or Workspace()
        self.verbose = verbose
        self._ser = None
        self._reader_thread = None
        self._running = False
        self._write_lock = threading.Lock()
        self._fb_lock = threading.Condition()
        self._fb: dict | None = None
        self._fb_seq = 0
        self._estop = threading.Event()
        self._sim_torque = True
        self.rate_hz = 40.0           # move_to() step rate
        self.speed_scale = 0.5        # operator speed setting; starts at 50 % every run
        self.sim_speed_mm_s = 250.0   # sim only: fake time for firmware moves (0 = instant)

    # ------------------------------------------------------------ connection
    def connect(self) -> "RoArm":
        if self.sim:
            self._fb = dict(T=FEEDBACK_REPLY, b=0.0, s=0.0, e=1.57, **HOME_POSE)
            self._fb_seq = 1
            self._log("SIM arm connected")
            return self
        if serial is None:
            raise ArmError("pyserial not installed: pip install pyserial")
        port = self.port or find_port()
        if port is None:
            raise ArmError("No RoArm serial port found. Pass --port /dev/ttyUSB0")
        self.port = port
        self._ser = serial.Serial(port, self.baud, timeout=0.1)
        self._ser.setDTR(False)
        self._ser.setRTS(False)
        time.sleep(2.0)  # the ESP32 may reboot when the port opens
        self._ser.reset_input_buffer()
        self._running = True
        self._reader_thread = threading.Thread(target=self._reader, daemon=True)
        self._reader_thread.start()
        self._log(f"Connected on {port} @ {self.baud}")
        return self

    def close(self) -> None:
        self._running = False
        if self._reader_thread:
            self._reader_thread.join(timeout=1.0)
        if self._ser:
            self._ser.close()

    def __enter__(self):
        return self.connect()

    def __exit__(self, *exc):
        self.close()

    def _reader(self) -> None:
        buf = b""
        while self._running:
            try:
                chunk = self._ser.read(256)
            except Exception:
                break
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode(errors="ignore").strip()
                if not text.startswith("{"):
                    continue
                try:
                    msg = json.loads(text)
                except json.JSONDecodeError:
                    continue
                if msg.get("T") == FEEDBACK_REPLY:
                    with self._fb_lock:
                        self._fb = msg
                        self._fb_seq += 1
                        self._fb_lock.notify_all()
                elif self.verbose:
                    print("arm <-", text)

    # ------------------------------------------------------------ low level
    def send(self, cmd: dict) -> None:
        line = json.dumps(cmd, separators=(",", ":"))
        if self.verbose:
            print("arm ->", line)
        if self.sim:
            self._sim_apply(cmd)
            return
        with self._write_lock:
            self._ser.write((line + "\n").encode())

    def feedback(self, timeout: float = 1.0) -> dict:
        """Request and return a fresh feedback message (x,y,z,t,b,s,e...)."""
        if self.sim:
            return dict(self._fb)
        with self._fb_lock:
            seq = self._fb_seq
        self.send({"T": CMD_FEEDBACK})
        deadline = time.time() + timeout
        with self._fb_lock:
            while self._fb_seq == seq:
                left = deadline - time.time()
                if left <= 0:
                    raise ArmError("No feedback from arm (check cable, port, baud)")
                self._fb_lock.wait(left)
            return dict(self._fb)

    def wait_alive(self, timeout: float = 10.0) -> dict:
        """After opening the port the arm's board may reboot and run its start-up
        move, ignoring requests for several seconds. Keep asking until it answers."""
        deadline = time.time() + timeout
        while True:
            try:
                return self.feedback(timeout=1.0)
            except ArmError:
                if time.time() > deadline:
                    raise ArmError("Arm's board connected over USB but not answering. "
                                   "Check the arm's power supply, then restart.")

    def pose(self) -> dict:
        fb = self.feedback()
        return {k: round(float(fb[k]), 2) for k in ("x", "y", "z", "t") if k in fb}

    # ------------------------------------------------------------ commands
    def torque(self, on: bool) -> None:
        """Torque OFF makes the arm limp - support it by hand first!"""
        self.send({"T": CMD_TORQUE, "cmd": 1 if on else 0})

    def home(self, wait: bool = True) -> None:
        self._guard()
        self.send({"T": CMD_HOME})
        if wait:
            self.wait_reached(HOME_POSE["x"], HOME_POSE["y"], HOME_POSE["z"], timeout=10)

    def gripper(self, rad: float, spd: float = 300, acc: float = 10, wait: bool = True) -> None:
        """Move the gripper at a limited speed.

        spd: servo steps/s (4096 steps = one turn; 0 would mean FULL speed, so
        it is never sent). acc: 0-254 in units of 100 steps/s^2.
        Waits roughly as long as the move takes.
        """
        self._guard()
        spd = max(1.0, float(spd))
        start_t = self._last_t()
        try:
            start_t = float(self.feedback(timeout=0.5).get("t", start_t))
        except ArmError:
            pass
        self.send({"T": CMD_GRIPPER_RAD, "cmd": round(rad, 3), "spd": int(spd), "acc": int(max(1, acc))})
        if wait:
            steps = abs(rad - start_t) * 4096 / (2 * math.pi)
            time.sleep(steps / spd + 0.25)

    def move_xyz(self, x: float, y: float, z: float, t: float | None = None,
                 spd: float = 0.25, wait: bool = True, tol: float = 6.0,
                 timeout: float = 8.0) -> None:
        """Firmware-interpolated move (T:104). Speed is a unitless firmware value
        and STOP only acts at the end of the move. Not used by the game any more;
        use move_to() instead."""
        self._guard()
        self.workspace.check(x, y, z)
        if t is None:
            t = self._last_t()
        self.send({"T": CMD_XYZT, "x": round(x, 1), "y": round(y, 1),
                   "z": round(z, 1), "t": round(t, 3), "spd": spd})
        if wait:
            self.wait_reached(x, y, z, tol=tol, timeout=timeout)

    def move_to(self, x: float, y: float, z: float, speed_mm_s: float = 40.0,
                accel_mm_s2: float = 100.0, tol: float = 6.0, settle_s: float = 3.0,
                scaled: bool = True) -> None:
        """Move with real speed limits, planned on the Pi.

        Path: interpolated around the arm's base (angle, radius, height), not a
        straight line. The base angle changes smoothly and never crosses the
        +-180 deg point behind the arm, so the base can't spin round, and a path
        never cuts through the base axis.

        Speed: trapezoid profile along the path, scaled by speed_scale, streamed
        as small T:1041 steps. Every step is checked: if one would move further
        than the speed allows (a sign of a wrong reading or a bad plan), the move
        is aborted before that step is sent. STOP acts within one step.
        """
        self._guard()
        self.workspace.check(x, y, z)
        start = self.feedback()
        x0, y0, z0 = float(start["x"]), float(start["y"]), float(start["z"])
        t = float(start.get("t", self._last_t()))
        k = self.speed_scale if scaled else 1.0
        v = max(1.0, speed_mm_s * k)
        a = max(10.0, accel_mm_s2 * k)

        path = BasePath((x0, y0, z0), (x, y, z))
        if path.length >= 0.5:
            profile = Trapezoid(path.length, v, a)
            period = 1.0 / self.rate_hz
            # Profile time advances by at most 2 ticks per step: after a hiccup on
            # the Pi the motion resumes where it was instead of jumping ahead.
            max_step = v * 2 * period * 1.2 + 1.0      # mm allowed between two steps
            prev = (x0, y0, z0)
            elapsed = 0.0
            last = time.monotonic()
            while True:
                if self._estop.is_set():
                    raise ArmError("E-stop active")
                now = time.monotonic()
                elapsed += min(now - last, 2 * period)
                last = now
                done = elapsed >= profile.duration
                p = path.point(path.length if done else profile.position(elapsed))
                jump = math.dist(prev, p)
                if jump > max_step:
                    self.estop()
                    raise ArmError(f"Safety stop: a single step of {jump:.0f} mm was planned "
                                   f"(limit {max_step:.0f} mm). That step was not sent; the arm is held where it is.")
                self.send({"T": CMD_XYZT_DIRECT, "x": round(p[0], 1), "y": round(p[1], 1),
                           "z": round(p[2], 1), "t": round(t, 3)})
                prev = p
                if done:
                    break
                time.sleep(max(0.0, period - (time.monotonic() - now)))
        self.wait_reached(x, y, z, tol=tol, timeout=settle_s)

    def move_xyz_stream(self, x: float, y: float, z: float, t: float | None = None,
                        speed_mm_s: float = 80.0, rate_hz: float = 40.0) -> None:
        """Kept for old scripts; same as move_to()."""
        self.move_to(x, y, z, speed_mm_s=speed_mm_s)

    def wait_reached(self, x: float, y: float, z: float, tol: float = 6.0,
                     timeout: float = 8.0) -> dict:
        deadline = time.time() + timeout
        last = None
        while time.time() < deadline:
            if self._estop.is_set():
                raise ArmError("E-stop active")
            try:
                last = self.feedback(timeout=1.0)
            except ArmError:
                continue  # firmware may be busy interpolating
            if math.dist((last["x"], last["y"], last["z"]), (x, y, z)) <= tol:
                return last
            time.sleep(0.05)
        where = f" (at {last['x']:.0f},{last['y']:.0f},{last['z']:.0f})" if last else ""
        raise ArmError(f"Did not reach ({x:.0f},{y:.0f},{z:.0f}) in {timeout}s{where}")

    # ------------------------------------------------------------ safety
    def estop(self) -> None:
        """Latch STOP: freeze at current position and refuse further moves."""
        self._estop.set()
        try:
            fb = self.feedback(timeout=0.5)
            self.send({"T": CMD_XYZT_DIRECT, "x": fb["x"], "y": fb["y"],
                       "z": fb["z"], "t": fb["t"]})
        except ArmError:
            pass

    def reset_estop(self) -> None:
        self._estop.clear()

    @property
    def stopped(self) -> bool:
        return self._estop.is_set()

    def _guard(self) -> None:
        if self._estop.is_set():
            raise ArmError("E-stop active - call reset_estop() first")

    # ------------------------------------------------------------ helpers
    def _last_t(self) -> float:
        if self._fb and "t" in self._fb:
            return float(self._fb["t"])
        return GRIPPER_CLOSED

    def _log(self, msg: str) -> None:
        print(f"[arm] {msg}")

    def _sim_apply(self, cmd: dict) -> None:
        T = cmd.get("T")
        fb = self._fb
        if T in (CMD_XYZT, CMD_XYZT_DIRECT):
            if self._sim_torque:
                if T == CMD_XYZT and self.sim_speed_mm_s:
                    d = math.dist((fb["x"], fb["y"], fb["z"]), (cmd["x"], cmd["y"], cmd["z"]))
                    time.sleep(d / self.sim_speed_mm_s)
                fb.update(x=cmd["x"], y=cmd["y"], z=cmd["z"], t=cmd["t"])
        elif T == CMD_HOME:
            fb.update(HOME_POSE)
        elif T == CMD_GRIPPER_RAD:
            fb["t"] = cmd["cmd"]
        elif T == CMD_TORQUE:
            self._sim_torque = bool(cmd["cmd"])
        self._fb_seq += 1

    def sim_set_pose(self, **kw) -> None:
        """Sim only: pretend the arm was moved by hand (for teach tests)."""
        self._fb.update(kw)
