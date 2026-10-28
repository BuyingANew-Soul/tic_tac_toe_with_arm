"""Tic-tac-toe robot: web server + game flow. One process runs everything.

    python3 -m ttt.server            # real arm (config.yaml)
    python3 -m ttt.server --sim      # simulated arm, no hardware

Pages:  /tablet  (students play)   /screen  (big screen)   /operator  (you)
"""
from __future__ import annotations

import argparse
import asyncio
import json
import logging
import random
import time
from collections import deque
from contextlib import asynccontextmanager
from pathlib import Path

import uvicorn
import yaml
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, RedirectResponse
from fastapi.staticfiles import StaticFiles

from . import poses as P
from .arm import ArmError, RoArm
from .game import HUMAN, MODES, ROBOT, Game, futures_checked
from .skills import SkillConfig, Skills

ROOT = Path(__file__).resolve().parents[1]
WEB = ROOT / "web"
log = logging.getLogger("ttt")

# Phases the pages react to
IDLE, STUDENT, THINKING, ROBOT_MOVE, RESULT, ASK_COLOUR, GIVING, ENJOY, STOPPED, CALIB = (
    "idle", "student", "thinking", "robot", "result", "ask_colour", "giving", "enjoy",
    "stopped", "calib")


class TouchWaiter:
    """Waits for the tablet to report a touch on an expected cell."""

    def __init__(self):
        self.cell: int | None = None
        self.event = asyncio.Event()
        self.seen: int | None = None

    def expect(self, cell: int):
        self.cell, self.seen = cell, None
        self.event.clear()

    def touched(self, cell: int) -> bool:
        if self.cell is not None and cell == self.cell:
            self.seen = cell
            self.event.set()
            return True
        return False

    def clear(self):
        self.cell = None


class TttApp:
    def __init__(self, cfg: dict, sim: bool):
        self.cfg = cfg
        self.sim = sim
        a = cfg["arm"]
        self.backend = "sim" if sim else a.get("backend", "serial")
        self.poses_file = ROOT / cfg["poses_file"]
        self.tray_file = ROOT / cfg["tray_file"]
        self.settings_file = ROOT / "data" / "settings.yaml"
        # each backend keeps its own calibration: positions from the arm's own
        # firmware (serial) and from MoveIt (ros) are in different frames
        prefix = {"sim": "sim_", "ros": "ros_"}.get(self.backend, "")
        if prefix:
            self.poses_file, self.tray_file, self.settings_file = (
                f.with_name(prefix + f.name) for f in (self.poses_file, self.tray_file, self.settings_file))
        self.poses = P.load(self.poses_file)
        if sim and not self.poses:
            self.poses = dict(P.SIM_DEFAULTS)
        ros_cfg = cfg.get("ros") or {}
        if self.backend == "ros":
            from .ros_arm import RosArm
            self.arm = RosArm(ros_cfg)
        else:
            self.arm = RoArm(port=a.get("port"), baud=a.get("baud", 115200), sim=sim)
        skill_cfg = dict(cfg.get("skills", {}))
        if self.backend == "ros":   # MoveIt gripper joint: 0 = closed, 1.5 = open
            skill_cfg["gripper_open"] = ros_cfg.get("gripper_open", 1.5)
            skill_cfg["gripper_grip"] = ros_cfg.get("gripper_grip", 0.4)
        skill_cfg.update(P.load(self.settings_file).get("skills") or {})
        self.skills = Skills(self.arm, self.poses, SkillConfig(**skill_cfg))
        self.arm.workspace.max_reach = float(a.get("max_reach_mm", 520))
        self.update_floor()
        self.colours: dict = cfg["colours"]
        tray = P.load(self.tray_file)
        self.tray_plan: dict[int, str | None] = tray.get("plan") or {}
        self.tray: dict[int, str | None] = tray.get("content") or {}

        self.game = Game(cfg["game"]["mode"])
        self.phase = IDLE
        self.message = "press_play"
        self.robot_cell: int | None = None
        self.futures = 0
        self.alert: str | None = None
        self.stats = {"student": 0, "robot": 0, "draw": 0, "chocolates": 0}
        self.lang = "fi"
        self.estopped = False
        self.prev_phase = IDLE
        self.resume_action = None
        self.pending_colour: str | None = None
        self.calib_pose: dict | None = None
        self.colour_choice: asyncio.Future | None = None
        self.torque_on = False
        self.reward_draw = bool(cfg["game"].get("reward_draw", True))
        self.robot_touch = cfg["game"].get("robot_move", "tap") != "point"

        self.clients: dict[WebSocket, str] = {}
        self.logs: deque = deque(maxlen=60)
        self.touch = TouchWaiter()
        self.arm_lock = asyncio.Lock()
        self.flow: asyncio.Task | None = None

    # ================================================================ plumbing
    async def startup(self):
        try:
            await asyncio.to_thread(self.arm.connect)
            fb = await asyncio.to_thread(self.arm.wait_alive, 10.0)
            await asyncio.to_thread(self.arm.torque, True)
            self.torque_on = True
            self.calib_pose = fb
            self.say(f"arm connected ({'SIM' if self.sim else self.arm.port}), "
                     f"position x {fb['x']:.0f} y {fb['y']:.0f} z {fb['z']:.0f}")
            await asyncio.to_thread(self.update_scene)
            # deliberately NO movement at start-up: the arm only moves when you press something
        except ArmError as e:
            self.raise_alert(f"Arm not connected: {e}")
        missing = P.missing(self.poses)
        if missing:
            self.raise_alert("Calibration needed: " + ", ".join(missing))

    def duplicate_of(self, name: str, xyz) -> str | None:
        """Another taught pose within 3 mm of xyz (usually means a frozen reading)."""
        others = {k: self.poses[k] for k in ("home", "cup") if k in self.poses}
        for group, prefix in (("board_corners", "corner_"), ("cells", "cell_"), ("tray", "tray_")):
            others.update({f"{prefix}{k}": v for k, v in (self.poses.get(group) or {}).items()})
        others.pop(name, None)
        for other, pos in others.items():
            if sum((a - b) ** 2 for a, b in zip(pos, xyz)) ** 0.5 < 3.0:
                return other
        return None

    def update_scene(self):
        """MoveIt only: put the tablet (from the taught corners) into the planning scene."""
        if hasattr(self.arm, "update_scene"):
            try:
                self.arm.update_scene(self.poses)
            except Exception as e:
                self.say(f"planning scene not updated: {e}")

    def update_floor(self):
        """Safety floor = just under the lowest taught position (minus the deepest tap)."""
        zs = []
        for key in ("home", "cup"):
            if key in self.poses:
                zs.append(self.poses[key][2])
        for group in ("board_corners", "cells", "tray"):
            zs += [p[2] for p in (self.poses.get(group) or {}).values()]
        if zs:
            t = self.cfg["touch"]
            margin = self.skills.cfg.press_mm + t["retries"] * t["retry_extra_mm"] + 10
            self.arm.workspace.z_floor = min(zs) - margin
        else:
            self.arm.workspace.z_floor = None

    def say(self, text: str):
        log.info(text)
        self.logs.append(time.strftime("%H:%M:%S ") + text)

    def raise_alert(self, text: str):
        self.alert = text
        self.say("ALERT: " + text)

    def state(self) -> dict:
        g = self.game
        show_scores = self.phase in (THINKING, ROBOT_MOVE)
        available = {}
        for colour in self.tray.values():
            if colour:
                available[colour] = available.get(colour, 0) + 1
        return dict(
            type="state", phase=self.phase, message=self.message, lang=self.lang,
            board=g.board, result=g.result, line=g.line_cells(), mode=g.mode,
            scores=g.scores_for_screen() if show_scores else {},
            robot_cell=self.robot_cell if self.phase == ROBOT_MOVE else None, futures=self.futures,
            reward_draw=self.reward_draw,
            robot_move="tap" if self.robot_touch else "point",
            speed_pct=round(self.arm.speed_scale * 100),
            stats=self.stats, alert=self.alert, estop=self.estopped, sim=self.sim, backend=self.backend,
            colours=self.colours, available=available,
            tray_plan=self.tray_plan, tray=self.tray,
            missing=P.missing(self.poses), logs=list(self.logs)[-15:],
            calib_pose=self.calib_pose,
            tray_slots=sorted((self.poses.get("tray") or {}).keys()),
            gripper=dict(open=self.skills.cfg.gripper_open, grip=self.skills.cfg.gripper_grip),
        )

    async def broadcast(self):
        msg = json.dumps(self.state())
        dead = []
        for ws in list(self.clients):
            try:
                await ws.send_text(msg)
            except Exception:
                dead.append(ws)
        for ws in dead:
            self.clients.pop(ws, None)

    def background(self, coro):
        task = asyncio.create_task(coro)
        task.add_done_callback(lambda t: t.cancelled() or (t.exception() and log.error(
            "background task failed", exc_info=t.exception())))
        return task

    def run_flow(self, coro):
        """Start the main game-flow task (only one at a time)."""
        if self.flow and not self.flow.done():
            self.flow.cancel()
        self.flow = asyncio.create_task(self._guard(coro))

    async def _guard(self, coro):
        try:
            await coro
        except asyncio.CancelledError:
            raise
        except ArmError as e:
            if not self.estopped:
                self.raise_alert(f"Arm error: {e}")
                await self.enter_stop(e_from_arm=True)
        except Exception as e:
            log.exception("flow crashed")
            self.raise_alert(f"Software error: {e}")
        finally:
            await self.broadcast()

    async def arm_do(self, fn, *args):
        if self.estopped:
            raise ArmError("E-stop active")
        async with self.arm_lock:
            if not self.torque_on:   # left limp after teaching by hand: hold position first
                await asyncio.to_thread(self.arm.torque, True)
                self.torque_on = True
                self.say("torque ON (automatically, before moving)")
            return await asyncio.to_thread(fn, *args)

    async def set_phase(self, phase: str, message: str | None = None):
        self.phase = phase
        if message:
            self.message = message
        await self.broadcast()

    # ================================================================ game flow
    async def new_game(self):
        if self.estopped:
            return
        needed = [m for m in P.missing(self.poses) if not m.startswith("tray")]
        if needed:   # the robot can't play without knowing where the board is
            self.raise_alert("Can't start a game yet - calibrate first: " + ", ".join(needed))
            await self.broadcast()
            return
        if self.flow and not self.flow.done():   # e.g. result screen timer still running
            self.flow.cancel()
        self.game.reset()
        self.robot_cell, self.futures, self.pending_colour = None, 0, None
        await self.set_phase(STUDENT, "your_turn")

    async def on_student_touch(self, cell: int):
        i = cell - 1
        if not self.game.human_move(i):
            return
        self.say(f"student X -> cell {cell}")
        if self.game.result:
            self.phase = RESULT
            self.message = {"X": "you_win", "draw": "draw"}.get(self.game.result, "robot_wins")
            self.run_flow(self.finish())
        else:
            self.run_flow(self.robot_turn())
        await self.broadcast()

    async def robot_turn(self):
        self.resume_action = self.robot_turn
        i, mistake = self.game.plan_robot()
        self.futures = futures_checked(self.game.board)
        self.robot_cell = i + 1
        await self.set_phase(THINKING, "thinking")
        await asyncio.sleep(self.cfg["game"]["think_s"])

        await self.set_phase(ROBOT_MOVE, "robot_moving")
        if self.robot_touch:
            confirmed = await self.tap_confirmed(self.robot_cell, self.cfg["touch"]["retries"])
        else:   # point mode: the robot points at its square, the game places the O
            await self.arm_do(self.skills.tap, self.robot_cell, 0.0, False)
            confirmed = True
        if not confirmed:
            self.raise_alert(f"Tablet did not register the tap on cell {self.robot_cell}. "
                             "Placed the move anyway - check the stylus.")
        self.game.apply_robot(i, mistake)
        self.say(f"robot O -> cell {self.robot_cell}" + (" (deliberate mistake)" if mistake else ""))
        self.robot_cell = None
        self.resume_action = None
        if self.game.result:
            await self.finish()
        else:
            await self.set_phase(STUDENT, "your_turn")
            self.background(self.safe_arm(self.skills.rest))

    async def tap_confirmed(self, cell: int, retries: int) -> bool:
        t = self.cfg["touch"]
        for attempt in range(retries + 1):
            self.touch.expect(cell)
            await self.arm_do(self.skills.tap, cell, attempt * t["retry_extra_mm"])
            try:
                await asyncio.wait_for(self.touch.event.wait(), t["timeout_s"])
                self.touch.clear()
                return True
            except asyncio.TimeoutError:
                self.say(f"no touch registered on cell {cell} (attempt {attempt + 1})")
        self.touch.clear()
        return False

    async def safe_arm(self, fn, *args):
        try:
            await self.arm_do(fn, *args)
        except ArmError as e:
            if not self.estopped:
                self.raise_alert(f"Arm error: {e}")

    async def finish(self):
        r = self.game.result
        key = {"X": "student", "O": "robot", "draw": "draw"}[r]
        self.stats[key] += 1
        self.say(f"game over: {key}")
        rewarded = r == HUMAN or (r == "draw" and self.reward_draw)
        if rewarded and any(self.tray.values()):
            await self.ask_and_give(draw=(r == "draw"))
            return
        if r == HUMAN:
            await self.set_phase(RESULT, "no_chocolate")
        elif r == ROBOT:
            await self.set_phase(RESULT, "robot_wins")
            await self.safe_arm(self.skills.wave)
        else:
            await self.set_phase(RESULT, "draw")
        # the result stays on screen until the next student presses Play

    async def ask_and_give(self, draw: bool = False):
        """Student won (or drew): ask for a colour (30 s, then pick one), then deliver."""
        self.resume_action = lambda: self.ask_and_give(draw)
        self.colour_choice = asyncio.get_running_loop().create_future()
        await self.set_phase(ASK_COLOUR, "draw_pick_colour" if draw else "pick_colour")
        try:
            colour = await asyncio.wait_for(self.colour_choice, 30)
        except asyncio.TimeoutError:
            colour = random.choice([c for c in self.tray.values() if c])
        await self.give(colour)

    async def give(self, colour: str):
        slot = next((s for s, c in sorted(self.tray.items()) if c == colour), None)
        if slot is None:
            return
        self.pending_colour = colour
        self.resume_action = lambda: self.give(colour)
        await self.set_phase(GIVING, "giving")
        await self.arm_do(self.skills.give, slot)
        self.tray[slot] = None
        self.save_tray()
        self.stats["chocolates"] += 1
        self.say(f"gave {colour} chocolate from slot {slot}")
        self.resume_action = None
        await self.set_phase(ENJOY, "enjoy")
        await asyncio.sleep(self.cfg["game"]["result_s"])
        await self.set_phase(IDLE, "press_play")

    # ================================================================ safety
    async def enter_stop(self, e_from_arm: bool = False):
        if self.phase != STOPPED:
            self.prev_phase = self.phase
        self.estopped = True
        await asyncio.to_thread(self.arm.estop)
        if self.flow and not self.flow.done() and asyncio.current_task() is not self.flow:
            self.flow.cancel()
        self.touch.clear()
        await self.set_phase(STOPPED, "stopped")

    async def resume(self):
        self.arm.reset_estop()
        self.estopped = False
        self.alert = None
        action = self.resume_action
        prev = self.prev_phase
        self.say("resumed")
        if action and prev in (THINKING, ROBOT_MOVE, ASK_COLOUR, GIVING):
            self.run_flow(action())
        elif prev == STUDENT:
            await self.set_phase(STUDENT, "your_turn")
        elif prev == RESULT:
            await self.set_phase(RESULT)
        else:
            await self.set_phase(IDLE, "press_play")
        self.background(self.safe_arm(self.skills.rest))

    # ================================================================ tray
    def save_tray(self):
        P.save({"plan": self.tray_plan, "content": self.tray}, self.tray_file)

    # ================================================================ calibration
    async def calib(self, msg: dict):
        if self.phase not in (IDLE, RESULT, CALIB) and msg["cmd"] not in ("pose",):
            self.say("calibration only when no game is running")
            return
        cmd = msg["cmd"]
        try:
            if cmd == "torque":
                await asyncio.to_thread(self.arm.torque, bool(msg["on"]))
                self.torque_on = bool(msg["on"])
                self.say("torque " + ("ON" if msg["on"] else "OFF - move the arm by hand"))
            elif cmd == "jog":
                await self.arm_do(self.skills.jog, msg["axis"], float(msg["mm"]))
            elif cmd == "save":
                p = await asyncio.to_thread(self.arm.pose)
                same = self.duplicate_of(msg["name"], [p["x"], p["y"], p["z"]])
                if same:
                    self.raise_alert(
                        f"Not saved: {msg['name']} would be the same position as {same}. "
                        "Did the arm move? If the position under the arrows doesn't change "
                        "when you move the arm, its servos aren't reporting: check the arm's "
                        "power supply and restart the program.")
                    self.calib_pose = p
                    await self.broadcast()
                    return
                P.set_pose(self.poses, msg["name"], [p["x"], p["y"], p["z"]])
                P.save(self.poses, self.poses_file)
                self.update_floor()
                await asyncio.to_thread(self.update_scene)
                self.say(f"saved {msg['name']} = {[p['x'], p['y'], p['z']]}")
                if not P.missing(self.poses) and self.alert and "Calibration" in self.alert:
                    self.alert = None
            elif cmd == "goto":
                await self.arm_do(self.skills.goto, P.get_pose(self.poses, msg["name"]))
            elif cmd == "tap":
                await self.arm_do(self.skills.tap, int(msg["cell"]))
            elif cmd == "grip":
                await self.arm_do(self.arm.gripper, float(msg["rad"]))
            elif cmd == "give":
                await self.arm_do(self.skills.give, int(msg["slot"]))
            elif cmd == "home":
                await self.arm_do(self.skills.rest)
            elif cmd == "tap_test":
                self.run_flow(self.tap_test(int(msg.get("repeats", 1))))
            elif cmd == "pose":
                pass
            self.calib_pose = await asyncio.to_thread(self.arm.pose)
        except (ArmError, KeyError, ValueError) as e:
            self.say(f"calibration error: {e}")
        await self.broadcast()

    async def tap_test(self, repeats: int):
        await self.set_phase(CALIB)
        hits = {c: 0 for c in range(1, 10)}
        for _ in range(repeats):
            for c in range(1, 10):
                ok = await self.tap_confirmed(c, retries=0)
                hits[c] += ok
                self.say(f"tap test cell {c}: {'HIT' if ok else 'MISS'}")
                await self.broadcast()
        bad = [c for c, h in hits.items() if h < repeats]
        self.say(f"tap test done: {sum(hits.values())}/{9 * repeats} hits" +
                 (f", check cells {bad}" if bad else " - all good"))
        await self.safe_arm(self.skills.rest)
        await self.set_phase(IDLE, "press_play")

    # ================================================================ messages
    async def handle(self, ws: WebSocket, role: str, msg: dict):
        t = msg.get("type")
        if t == "touch":
            cell = int(msg["cell"])
            if self.touch.touched(cell):
                return
            if self.phase == STUDENT:
                await self.on_student_touch(cell)
        elif t == "play" and self.phase in (IDLE, RESULT):
            await self.new_game()
        elif t == "colour" and self.phase == ASK_COLOUR:
            fut = self.colour_choice
            if fut and not fut.done() and msg.get("colour") in self.state()["available"]:
                fut.set_result(msg["colour"])
        elif role == "operator":
            await self.handle_operator(msg)

    async def handle_operator(self, msg: dict):
        t = msg.get("type")
        if t == "stop":
            await self.enter_stop()
        elif t == "resume" and self.estopped:
            await self.resume()
        elif t == "mode" and msg.get("mode") in MODES:
            self.game.mode = msg["mode"]
            self.say(f"mode -> {msg['mode']}")
        elif t == "pose_poll":   # live position while the calibration panel is open
            if self.phase in (IDLE, RESULT, CALIB, STOPPED) and not self.arm_lock.locked():
                try:
                    self.calib_pose = await asyncio.to_thread(self.arm.feedback, 0.5)
                except ArmError:
                    self.calib_pose = None
        elif t == "speed" and msg.get("pct") in (10, 25, 50, 100):
            self.arm.speed_scale = msg["pct"] / 100
            self.say(f"arm speed {msg['pct']} %")
        elif t == "robot_move" and msg.get("mode") in ("tap", "point"):
            self.robot_touch = msg["mode"] == "tap"
            self.say("robot moves by " + ("touching the screen" if self.robot_touch else "pointing (no touch)"))
        elif t == "reward_draw":
            self.reward_draw = bool(msg.get("on"))
            self.say("chocolate for draws " + ("ON" if self.reward_draw else "OFF"))
        elif t == "lang":
            self.lang = msg.get("lang", "fi")
        elif t == "reset":
            if self.flow and not self.flow.done():
                self.flow.cancel()
            self.game.reset()
            self.touch.clear()
            self.robot_cell = None
            if not self.estopped:
                self.phase, self.message = IDLE, "press_play"
                self.background(self.safe_arm(self.skills.rest))
        elif t == "clear_alert":
            self.alert = None
        elif t == "reset_stats":
            self.stats = {k: 0 for k in self.stats}
        elif t == "tray_plan":
            self.tray_plan[int(msg["slot"])] = msg.get("colour") or None
            self.save_tray()
        elif t == "refill":
            self.tray = {s: c for s, c in self.tray_plan.items()}
            self.save_tray()
            self.say("tray refilled")
        elif t == "set_gripper":
            for k in ("open", "grip"):
                if msg.get(k) is not None:
                    setattr(self.skills.cfg, "gripper_" + k, float(msg[k]))
            P.save({"skills": {"gripper_open": self.skills.cfg.gripper_open,
                               "gripper_grip": self.skills.cfg.gripper_grip}}, self.settings_file)
            self.say(f"gripper open={self.skills.cfg.gripper_open} grip={self.skills.cfg.gripper_grip} saved")
        elif t == "calib":
            await self.calib(msg)
            return
        await self.broadcast()


# ==================================================================== web app
def build(cfg: dict, sim: bool) -> FastAPI:
    ttt = TttApp(cfg, sim)

    @asynccontextmanager
    async def lifespan(_app):
        await ttt.startup()
        yield
        ttt.arm.close()

    app = FastAPI(lifespan=lifespan)
    app.state.ttt = ttt

    @app.get("/")
    async def index():
        return RedirectResponse("/operator")

    for page in ("tablet", "screen", "operator"):
        def make(p=page):
            async def route():
                return FileResponse(WEB / f"{p}.html")
            return route
        app.get(f"/{page}")(make())

    app.mount("/static", StaticFiles(directory=WEB), name="static")

    @app.websocket("/ws")
    async def ws_endpoint(ws: WebSocket, role: str = "tablet"):
        await ws.accept()
        ttt.clients[ws] = role
        await ws.send_text(json.dumps(ttt.state()))
        try:
            while True:
                msg = json.loads(await ws.receive_text())
                await ttt.handle(ws, role, msg)
        except WebSocketDisconnect:
            pass
        except Exception as e:   # a page closed mid-message: not an error worth a traceback
            log.debug("websocket closed: %s", e)
        finally:
            ttt.clients.pop(ws, None)

    return app


def load_config(path: Path) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--sim", action="store_true", help="simulated arm")
    ap.add_argument("--backend", choices=["serial", "ros", "sim"],
                    help="override arm.backend from config.yaml")
    ap.add_argument("--config", default=str(ROOT / "config.yaml"))
    ap.add_argument("--port", type=int, default=8000)
    a = ap.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
    cfg = load_config(Path(a.config))
    if a.backend:
        cfg["arm"]["backend"] = a.backend
    sim = a.sim or cfg["arm"]["backend"] == "sim"
    print_addresses(a.port, sim)
    uvicorn.run(build(cfg, sim), host="0.0.0.0", port=a.port, log_level="warning")


def print_addresses(port: int, sim: bool):
    """Show the page addresses with this machine's real IP, so nobody has to look it up."""
    import socket
    ip = "localhost"
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))   # no packet is sent; just picks the LAN interface
        ip = s.getsockname()[0]
        s.close()
    except OSError:
        pass
    base = f"http://{ip}:{port}"
    print(f"""
  Robot tic-tac-toe is running{'  (SIMULATED ARM)' if sim else ''}
  ------------------------------------------------------------
  Operator (you):   {base}/operator
  Tablet (players): {base}/tablet
  Big screen:       {base}/screen
  ------------------------------------------------------------
  Stop the program with Ctrl+C
""", flush=True)


if __name__ == "__main__":
    main()
