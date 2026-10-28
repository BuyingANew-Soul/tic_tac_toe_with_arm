"""End-to-end check against the simulated arm.

Starts the server in-process, then a fake tablet plays games while a fake
operator configures the tray, switches modes and presses STOP mid-move.
The fake tablet only "registers" a robot tap when the simulated arm's
stylus actually reaches the target cell's taught height.

    python3 tests/e2e_sim.py
"""
import asyncio
import json
import random
import sys
import threading
import time
from pathlib import Path

import uvicorn
import websockets

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from ttt import poses as P  # noqa: E402
from ttt.server import build, load_config  # noqa: E402

PORT = 8765
cfg = load_config(ROOT / "config.yaml")
cfg["game"].update(think_s=0.2, result_s=0.3)
cfg.setdefault("skills", {}).update(travel_mm_s=800, near_mm_s=300, accel_mm_s2=5000, gripper_spd=4000)  # fast sim only
cfg["poses_file"] = "data/test_poses.yaml"
cfg["tray_file"] = "data/test_tray.yaml"
for f in ("data/test_poses.yaml", "data/test_tray.yaml", "data/sim_test_poses.yaml", "data/sim_test_tray.yaml"):
    (ROOT / f).unlink(missing_ok=True)

app = build(cfg, sim=True)
ttt = app.state.ttt
threading.Thread(target=lambda: uvicorn.run(app, port=PORT, log_level="warning"), daemon=True).start()


def stylus_on(cell):
    """True when the sim arm's tip is at/below the cell's touch height."""
    fb = ttt.arm._fb
    x, y, z = P.cell_xyz(ttt.poses, cell)
    return abs(fb["x"] - x) < 2 and abs(fb["y"] - y) < 2 and fb["z"] <= z + 0.5


class Client:
    def __init__(self, role):
        self.role, self.state, self.states = role, None, []

    async def connect(self):
        for _ in range(100):
            try:
                self.ws = await websockets.connect(f"ws://127.0.0.1:{PORT}/ws?role={self.role}")
                break
            except OSError:
                await asyncio.sleep(0.1)
        asyncio.create_task(self._rx())

    async def _rx(self):
        async for m in self.ws:
            self.state = json.loads(m)
            self.states.append(self.state)

    async def send(self, **msg):
        await self.ws.send(json.dumps(msg))

    async def wait(self, cond, timeout=20.0, what=""):
        end = time.time() + timeout
        while time.time() < end:
            if self.state and cond(self.state):
                return self.state
            await asyncio.sleep(0.02)
        raise AssertionError(f"timeout waiting for {what}; phase={self.state and self.state['phase']}")


async def stylus_watcher(tab, stop, miss_cells=()):
    """Plays the role of the tablet hardware: report touches the stylus makes."""
    pressed = None
    while not stop.is_set():
        hit = next((c for c in range(1, 10) if stylus_on(c)), None)
        if hit and hit != pressed and hit not in miss_cells:
            await tab.send(type="touch", cell=hit)
        pressed = hit
        await asyncio.sleep(0.01)


async def play_game(tab, rng, sharp=True):
    await tab.wait(lambda s: s["phase"] in ("idle", "result"), what="idle")
    await tab.send(type="play")
    await tab.wait(lambda s: s["phase"] == "student" and not any(s["board"]), what="new game")
    while True:
        s = await tab.wait(lambda s: s["phase"] in ("student", "result", "ask_colour"), what="student turn")
        if s["phase"] != "student":
            return s
        b = s["board"]
        empty = [i for i in range(9) if b[i] is None]
        move = None
        if sharp:   # take a win, else block, else random
            from ttt.game import winner
            for who in ("X", "O"):
                for i in empty:
                    bb = list(b); bb[i] = who
                    if winner(bb) == who:
                        move = move if move is not None else i
        move = rng.choice(empty) if move is None else move
        n = sum(1 for v in b if v)
        await tab.send(type="touch", cell=move + 1)
        await tab.wait(lambda s: sum(1 for v in s["board"] if v) > n or s["result"], what="move applied")


async def main():
    rng = random.Random(3)
    tab, op = Client("tablet"), Client("operator")
    await tab.connect(); await op.connect()
    stop = asyncio.Event()
    watcher = asyncio.create_task(stylus_watcher(tab, stop))
    await op.wait(lambda s: True)

    # tray: 3 slots, two colours
    for slot, colour in [(1, "red"), (2, "blue"), (3, "red")]:
        await op.send(type="tray_plan", slot=slot, colour=colour)
    await op.send(type="refill")
    await op.wait(lambda s: s["available"] == {"red": 2, "blue": 1}, what="tray refill")
    print("tray ok:", op.state["available"])

    # 1) hard mode: robot must never lose
    await op.send(type="mode", mode="hard")
    await op.send(type="reward_draw", on=False)
    results = []
    for _ in range(4):
        s = await play_game(tab, rng)
        results.append(s["result"])
        assert s["result"] != "X", "hard mode lost!"
    print("hard games:", results, "stats:", op.state["stats"])
    assert op.state["stats"]["draw"] + op.state["stats"]["robot"] == 4
    assert not op.state["alert"], op.state["alert"]
    leaks = [x for x in op.states if x["phase"] == "thinking" and x["robot_cell"]]
    assert not leaks, "robot's move was revealed while thinking"
    seen_robot = [x for x in op.states if x["phase"] == "robot" and x["robot_cell"]]
    assert seen_robot, "target never shown during the robot's move"
    await asyncio.sleep(1.0)
    assert op.state["phase"] == "result" and op.state["message"] == {"draw": "draw", "O": "robot_wins"}[results[-1]], (op.state["phase"], op.state["message"], results, op.state["board"], op.state["result"])
    stale = [x for x in op.states if x["result"] and x["phase"] == "student"]
    assert not stale, "'your turn' shown after the game ended"
    print("no early highlight, draw message stays up: ok")

    # 1b) draw with reward on -> colour question with the draw message
    await op.send(type="reward_draw", on=True)
    for _ in range(6):                      # the hard robot sometimes wins; play until a draw
        s = await play_game(tab, rng)
        if s["phase"] == "ask_colour":
            break
    assert s["phase"] == "ask_colour" and s["message"] == "draw_pick_colour", (s["phase"], s["message"])
    await tab.send(type="colour", colour="red")
    await tab.wait(lambda s: s["phase"] == "enjoy", timeout=30, what="enjoy after draw")
    print("draw rewarded:", op.state["available"])
    await tab.wait(lambda s: s["phase"] == "idle", what="idle after draw chocolate")
    await op.send(type="refill")
    await op.send(type="reward_draw", on=False)

    # 2) gift mode until the student wins, then choose blue
    await op.send(type="mode", mode="gift")
    for _ in range(10):
        s = await play_game(tab, rng)
        if s["phase"] == "ask_colour":
            break
        await tab.wait(lambda s: s["phase"] in ("idle", "result"), what="back to idle")
    assert s["phase"] == "ask_colour", "student never won in gift mode"
    assert s["line"], "no winning line sent"
    print("student won, line", s["line"], "- choosing blue")
    await tab.send(type="colour", colour="blue")
    await tab.wait(lambda s: s["phase"] == "giving", what="giving")
    await tab.wait(lambda s: s["phase"] == "enjoy", timeout=30, what="enjoy")
    assert op.state["available"] == {"red": 2}, op.state["available"]
    assert op.state["tray"]["2"] is None
    print("chocolate delivered; tray now", op.state["available"], "stats", op.state["stats"])

    # 3) STOP during the robot's tap, then resume - game must continue cleanly
    await tab.wait(lambda s: s["phase"] == "idle", what="idle after enjoy")
    await op.send(type="mode", mode="hard")
    await tab.send(type="play")
    await tab.wait(lambda s: s["phase"] == "student", what="student")
    await tab.send(type="touch", cell=5)
    await tab.wait(lambda s: s["phase"] == "robot", what="robot moving")
    await asyncio.sleep(0.15)
    await op.send(type="stop")
    s = await op.wait(lambda s: s["phase"] == "stopped", what="stopped")
    board_at_stop = list(s["board"])
    await asyncio.sleep(0.5)
    await tab.send(type="touch", cell=1)        # must be ignored while stopped
    await asyncio.sleep(0.2)
    assert op.state["board"] == board_at_stop, "board changed while stopped"
    await op.send(type="resume")
    s = await tab.wait(lambda s: s["phase"] == "student" and s["board"].count("O") == 1,
                       what="robot move completed after resume")
    print("stop/resume ok, board", s["board"])
    await op.send(type="reset")
    await tab.wait(lambda s: s["phase"] == "idle", what="reset")

    # 4) stylus misses on cell 7 for a whole game -> retry, then operator alert
    stop.set(); await watcher; stop.clear()
    watcher = asyncio.create_task(stylus_watcher(tab, stop, miss_cells=set(range(1, 10))))
    await tab.send(type="play")
    await tab.wait(lambda s: s["phase"] == "student", what="student")
    await tab.send(type="touch", cell=5)
    s = await op.wait(lambda s: s["alert"] and "did not register" in s["alert"], timeout=30, what="miss alert")
    print("missed-tap alert ok:", s["alert"][:60], "...")
    stop.set(); await watcher
    await op.send(type="clear_alert")
    await op.send(type="reset")

    # 5) calibration tap test with a working stylus
    stop.clear(); watcher = asyncio.create_task(stylus_watcher(tab, stop))
    await op.wait(lambda s: s["phase"] == "idle", what="idle")
    await op.send(type="calib", cmd="tap_test", repeats=1)
    s = await op.wait(lambda s: any("tap test done" in l for l in s["logs"]), timeout=60, what="tap test")
    print([l for l in s["logs"] if "tap test done" in l][-1])
    stop.set(); await watcher

    print("\nALL E2E CHECKS PASSED. stats:", op.state["stats"])


asyncio.run(main())
for f in ("data/test_poses.yaml", "data/test_tray.yaml", "data/sim_test_poses.yaml", "data/sim_test_tray.yaml"):
    (ROOT / f).unlink(missing_ok=True)
