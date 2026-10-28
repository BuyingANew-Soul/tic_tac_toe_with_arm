# Robot tic-tac-toe — Kokkola Material Week

A RoArm-M2 plays tic-tac-toe with students on a tablet. If a student wins or
draws, the arm gives them a chocolate in the colour they choose.

The arm moves through **ROS 2 Jazzy + MoveIt**. All devices plug into the
Pi; everything else runs on the laptop:

```
 Pi 5 — drivers only                         Laptop — brains
 ───────────────────                         ─────────────────────────────────────
 ROS_DOMAIN_ID 43 (game)
   roarm_driver ── /dev/roarm_game ◄─────── MoveIt + game server (ttt_laptop.launch.py)
                    (table arm)                 ▲
 ROS_DOMAIN_ID 42 (UGV, reward — later)         └── web pages: tablet, big screen, operator
   roarm_driver ── /dev/roarm_ugv  ◄─────── your AUWO excavation stack
   OAK-D Pro driver
```

The two arms use the same topic names, so each lives in its own ROS domain.
Game commands can't reach the bucket arm, and the other way round. (Tested:
during a full calibration and game, the bucket arm's driver received 0 commands.)

The game server only talks to MoveIt, the same way your excavation server
does: IK from `/compute_ik`, then a joint-space goal to `/move_action`. MoveIt
plans within the joint limits and around the tablet, which the game adds to
the planning scene as a box from your taught board corners.

**Contents**

- [Quick reference](#quick-reference)
- [Part 1 — Install (once)](#part-1--install-once)
- [Part 2 — Try it without the arm](#part-2--try-it-without-the-arm)
- [Part 3 — Start the real system](#part-3--start-the-real-system)
- [Part 4 — Set up the table and calibrate](#part-4--set-up-the-table-and-calibrate)
- [Part 5 — On the event day](#part-5--on-the-event-day)
- [Troubleshooting](#troubleshooting)
- [Settings and files](#settings-and-files)

---

## Quick reference

**Start, in this order:**

1. Arms powered on. They move to their start pose by themselves.
2. **Laptop**, in the `ttt` folder, with ROS and your RoArm workspace sourced:
   `ros2 launch launch/ttt_laptop.launch.py` → MoveIt + the game (domain 43)
3. **Pi**: `ros2 launch ~/ttt/launch/ttt_pi.launch.py` → both arm drivers + OAK-D
   (for the game alone: `ugv:=false camera:=false`)

The game prints the page addresses with the laptop's IP when it starts:

| Page | Address | Open it on | Used by |
|---|---|---|---|
| Operator | `http://<laptop-ip>:8000/operator` | your phone or the laptop | you: STOP, speed, difficulty, tray, calibration |
| Tablet | `http://<laptop-ip>:8000/tablet` | the Lenovo tablet | students play here |
| Big screen | `http://<laptop-ip>:8000/screen` | the computer on the big screen | the whole group watches |

**Stop:** Ctrl+C on the Pi first, then on the laptop.

**Emergency:** the red **STOP** on the operator page cancels the MoveIt motion
and holds the arm. Pulling the arm's power plug is the real emergency stop.

**Looking at the game's ROS topics by hand:** use `ROS_DOMAIN_ID=43`, for
example `ROS_DOMAIN_ID=43 ros2 topic echo /roarm/joint_states`. Your usual
shell is probably on 42 (the AUWO stack).

---

## Part 1 — Install (once)

**No new ROS workspace or package is needed.** `ttt` is a plain Python
program plus two launch files that ROS runs straight from their path. The
only ROS workspace involved is your existing RoArm one: its `roarm_driver`
runs on the Pi, its `roarm_moveit` config on the laptop.

| | Pi 5 | Laptop |
|---|---|---|
| Runs | the 3 device drivers | MoveIt, the game server, the web pages |
| Needs | ROS 2 Jazzy, your RoArm workspace (`roarm_driver`), the `ttt` folder | ROS 2 Jazzy + MoveIt, your RoArm workspace (`roarm_moveit`), the `ttt` folder, 3 Python packages |

### Laptop

1. **Put the project in your home folder:**
   ```bash
   cd ~ && unzip ttt-tictactoe.zip      # creates ~/ttt
   ```
2. **Install the web server packages** into the Python that ROS uses:
   ```bash
   sudo apt install python3-pip         # if pip is missing
   pip install --break-system-packages fastapi "uvicorn[standard]" pyyaml
   ```
3. **Check the game runs** (simulated arm, no ROS needed):
   ```bash
   cd ~/ttt && python3 -m ttt.server --sim
   ```
   Open the operator address it prints, then stop it with Ctrl+C.
4. **Joint velocity limits in your MoveIt config.** MoveIt on Jazzy refuses to
   plan without them ("No velocity limit was defined for joint ..."). Open
   `roarm_moveit/config/joint_limits.yaml` in your RoArm workspace. If it says
   `has_velocity_limits: false`, replace it with `~/ttt/launch/joint_limits.example.yaml`
   (1 rad/s, 2 rad/s² per joint) and rebuild that workspace.
5. **Check the names in `~/ttt/config.yaml` → `ros:`** match your MoveIt
   config: planning group (`arm`, or `hand` in Waveshare's original), gripper
   group, `hand_tcp`, `base_link`, and the joint states topics. Your SRDF
   (`roarm_moveit/config/*.srdf`) lists the groups.

### Pi

1. **Copy the project to the Pi** (from the laptop):
   ```bash
   scp -r ~/ttt <user>@<pi-ip>:~/
   ```
   Only `launch/` and `tools/` are used there.
2. **Your RoArm workspace with `roarm_driver`** must be built on the Pi (it
   already is). For the reward later, your AUWO-testbed workspace with
   `oak_d_bringup` is needed too.
3. **Recommended: cap the servo speed in the driver.** In
   `roarm_driver/roarm_driver.py` (in the Pi's RoArm workspace) change:
   ```python
               'spd': 1000,   # was 0 (= max). 1000 steps/s ≈ 88°/s, above MoveIt's 57°/s limit
               'acc': 10
   ```
   then rebuild: `colcon build --packages-select roarm_driver`.
   Why: the driver sends every position at **full servo speed**. Normally the
   positions are close together (in testing, at most 1.7° apart, 20 per
   second), but if MoveIt and the arm ever disagree, the servos jump at full speed.
4. **Give the two arms fixed names.** `/dev/ttyUSB0` and `/dev/ttyUSB1` can swap
   after a reboot, and then game moves would go to the bucket arm. Decide
   which Pi USB port each arm uses, label them, then:
   ```bash
   bash ~/ttt/tools/setup_arm_ports.sh
   ```
   It asks you to unplug and replug each arm, then creates `/dev/roarm_game`
   and `/dev/roarm_ugv`. Check with `ls -l /dev/roarm_*`. From now on,
   **always plug each arm into its own port**.

> For later code updates, replace the changed files and restart. `pip install`
> is only needed again if the dependencies change.

---

## Part 2 — Try it without the arm

The game also runs with a built-in simulated arm, without ROS:

1. `python3 -m ttt.server --sim`
2. Open the **operator** page, the **tablet** page and the **big screen** page.
3. Under **Chocolate tray**, choose a colour for each slot, then **Refill tray**.
4. On the tablet, press **Pelaa** and play. Try **Gift** mode to win a chocolate.
5. Press **STOP** during a robot move, then **Resume**.

To see the real MoveIt planning without the real arm, run the laptop launch
file without starting the driver on the Pi: MoveIt then moves only its
simulated arm (visible in RViz).

Each mode keeps its own calibration (`data/sim_*.yaml`, `data/ros_*.yaml`),
so trying things never changes your real setup.

---

## Part 3 — Start the real system

### Why the start order matters

Your `roarm_driver` is **open-loop**: it sends MoveIt's joint positions to the
servos and never reads the real positions back. MoveIt therefore *assumes*
the arm is where its own state says. Right after launching, that is the
start pose (base 0, shoulder 0, elbow 90°), which is also where the arm goes
by itself when powered on. If the two disagree when the driver starts, the
arm drives to MoveIt's position immediately.

So:

1. **Power-cycle the table arm** if it isn't in its start pose. It returns there by itself.
2. Start the **laptop** launch file and wait for "Robot tic-tac-toe is running".
3. Start the **Pi** launch file.
4. The game never moves the arm at startup. On the operator page, try one
   jog (Calibration → Z+ up, 5 mm) before anything else.

If you restart the laptop side while the arm is elsewhere: stop the Pi
launch, power-cycle the arm, then start again in the order above.

If one arm is unplugged, only its driver stops (with "could not open port");
the other arm keeps working.

**Teaching is by jogging only.** "Torque off and move by hand" isn't
available with MoveIt: the driver doesn't read the servos, so MoveIt wouldn't
know the arm moved and would snap it back on the next move.

### Safety rules for testing

- **The arm never moves on its own when the game starts.** Only buttons and games move it.
- **Every start is at 50 % speed** (Arm speed, top of the operator page).
  100 % = MoveIt velocity scaling 0.5 of the joint limits.
- **After any code update, test with the tablet out of reach first**, using
  **Point only** mode.
- Keep a hand near **STOP**, and the arm's power plug within reach.

---

## Part 4 — Set up the table and calibrate

Do this once at home, and again at the venue after the table is set up.
Everything is on the **operator page → Calibration**. Calibration only works
when no game is running. All values are saved in the `data/` folder and
survive restarts.

### 4.1 Mount the stylus

Fix the stylus to the **outside of the gripper's fixed jaw**, with the tip
**at least 1.5 cm below the jaws** (2 cm is good). The gripper must stay free
to pick chocolates. The tip must stick out that far because MoveIt keeps the
gripper's 3D model 30 mm above the tablet box, so the stylus is the only
part that reaches the screen.

### 4.2 Place the tablet

1. Put the tablet in a holder within the arm's reach and **tape the holder
   to the table**. If it moves, you must calibrate again.
2. On the tablet, open the **tablet page** and tap **⛶** for full screen.
3. In the tablet's settings: lock screen rotation, and set the screen to
   stay awake.
4. Touch the board with the stylus **held by hand**. If the square doesn't
   react, fix the stylus first (see [Troubleshooting](#troubleshooting)).

### 4.3 Teach the board corners

Moving the arm: use the arrow buttons with a step of 20, 5 or 1 mm. The
current position (`hand_tcp` in `base_link`, mm) is shown under the buttons.
Torque off isn't available with MoveIt (see Part 3).

After each saved corner, the tablet box in MoveIt is updated. In RViz you
can see it as a grey box under the board.

1. Move the stylus tip until it **just touches the centre of square 1**
   (top left as seen on the tablet). The square on the tablet should light up.
2. Press **Cell 1**.
3. Repeat for **Cell 3** (top right), **Cell 7** (bottom left) and **Cell 9**
   (bottom right).

The other five squares are calculated from these four.

```
 1 | 2 | 3
---+---+---
 4 | 5 | 6
---+---+---
 7 | 8 | 9
```

### 4.4 Teach home and cup

- **Home:** a resting spot beside the board, not above it, so students can
  see and reach every square. Keep it **low and close** to the board (roughly
  10–15 cm above it, off to the robot's side). The arm returns home after
  every move, so a far-away home (for example pointing straight up) makes
  every robot move several seconds slower. Move there, press **Home**.
- **Cup:** the gripper just above the prize cup. Move there, press **Cup**.

### 4.5 Check the taps

1. Make sure the tablet page is open.
2. Tap single squares with the **1–9** buttons under "Tap a cell".
3. Press **Tap test all 9**. The log shows HIT or MISS for every square.
   **All 9 must be HIT.** If one misses, see [Troubleshooting](#troubleshooting).

### 4.6 Tune the gripper

1. Type a value in **Gripper open** and press **Try**. Find the value where
   the jaws open wide enough for a chocolate.
2. Put a chocolate between the jaws. Type a value in **Gripper grip** and
   press **Try**. Find the value that holds it firmly without crushing it.
3. Press **Save gripper values**.

### 4.7 Teach the tray slots

For each chocolate position in the tray:

1. Place a chocolate in the slot.
2. Move the open gripper around it, at grasp height.
3. Type the slot number in the box (1, 2, 3, …) and press **Tray slot**.

Then type a slot number next to **Test give from slot** and press it. The arm
should pick that chocolate and drop it into the cup.

When nothing is left to teach, the Calibration section says
**Missing: nothing – ready**.

### 4.8 Fill the tray

1. Under **Chocolate tray**, choose the colour for each slot. You only need
   to do this once.
2. Put the chocolates in the real tray to match.
3. Press **Refill tray**.

---

## Part 5 — On the event day

### Before the first group

1. Start everything in the order from the [Quick reference](#quick-reference).
2. Open the three pages: tablet (full screen), big screen, and the operator
   page on your phone.
3. If anything moved since calibration, redo [Part 4](#part-4--set-up-the-table-and-calibrate).
   Run **Tap test all 9** in any case.
4. Fill the tray and press **Refill tray**.
5. Choose the difficulty and whether draws earn a chocolate.
6. Choose the screen language: **Suomi** (default) or **English**.

### During a group (about 10 minutes)

1. A student presses **Pelaa** on the tablet and plays as **X**. The robot
   plays **O**.
2. While the robot thinks, the big screen shows how many games it played out
   in its head and its verdict on each free square. Then the arm taps its move.
3. If the student wins or draws, the tablet asks which colour chocolate they
   want, and the arm drops it into the cup.
4. The result stays on screen until the next student presses **Pelaa**.

Each game takes about a minute, so 4–6 students can play per group.

**Difficulty** (switch any time, even mid-game):

| Mode | What it does |
|---|---|
| Hard | never loses: good for explaining how the robot "thinks" |
| Easy | sometimes makes one mistake: a careful student wins about 40% of games |
| Gift | lets the student win: use it when someone should get a chocolate |

**Chocolate for a draw: Yes / No.** In Hard mode most attentive students
draw, so turn this off if you're running low on chocolates.

**Robot's move: Touch the screen / Point only.** With *Point only*, the arm
moves above its square and points down (stopping 12 mm above the screen),
and the game places its O. Nothing touches the tablet, so there's no risk of
scratching and small calibration errors don't matter. Use it if tapping
isn't reliable on the day. The tap test and the calibration tap buttons
always touch the screen.

### Between groups

- Refill the tray and press **Refill tray**.
- **Cancel game** clears the board if a game is unfinished.

### If something goes wrong during a game

- Press **STOP**. The arm freezes and the screens say it has been stopped.
- Fix the problem, then press **Resume**. A half-finished robot move or
  chocolate delivery continues where it stopped.
- If a tap doesn't register, the robot retries once with a slightly deeper
  press. If it still misses, an orange alert appears on the operator page.

The **Log** at the bottom of the operator page lists every move, including
the robot's deliberate mistakes.

---

## Troubleshooting

| Problem | What to do |
|---|---|
| Pages say "reconnecting" | Check the program is running and the device is on the same Wi-Fi. They reconnect by themselves. |
| "No supported WebSocket library" in the terminal | `pip install 'uvicorn[standard]'` |
| "MoveIt not found" | Is the laptop launch running? On both machines, `ROS_DOMAIN_ID` must be the same. Check `ros2 action list` shows `/move_action`. |
| "No velocity limit was defined for joint" in the MoveIt log, every move fails | Add joint velocity limits, see Part 1. |
| "No arm position reaches (…)" when tapping | The tablet box is in the way: check the four corners are correct, that the stylus sticks out 1.5 cm or more, or raise `ros.tablet_below_mm`. |
| Arm moves in MoveIt/RViz but not in reality | The Pi driver isn't receiving: check it's running, and that its topic matches what MoveIt publishes (`/joint_states` or your remap). |
| Arm jumps when the driver starts | MoveIt's state and the arm disagreed: follow the start order in Part 3, and cap `spd` in the driver. |
| Arm test prints "No feedback from arm" | The port is busy, wrong, or the arm isn't powered. If it still fails, send the output of the test with `verbose=True`. |
| Arm connects but doesn't move | The arm's firmware may use different command numbers; they are at the top of `ttt/arm.py`. |
| Tablet doesn't register the robot's taps | Run a thin wire from the stylus's metal body to the arm's metal frame. Or raise `press_mm` in `config.yaml` (3 → 5) and restart. Run the tap test again. |
| One square is a few mm off | Re-teach the four corners. If only one square is off, move to it and add it to `data/poses.yaml` under `cells:` (for example `5: [240.0, 0.0, 12.0]`). |
| Chocolate slips out | Lower the **Gripper grip** value a little (closes tighter), then **Save gripper values**. |
| Calibration buttons are greyed out | A game is running, or the arm is stopped. Press **Cancel game** or **Resume**. |

---

## Settings and files

Most things are set on the operator page. `config.yaml` holds the rest;
restart the program after editing it. Settings missing from `config.yaml`
use the defaults below, so you only need to add the ones you want to change,
under the right heading (`skills:` or `game:`). The old `speed` and
`slow_speed` settings are ignored now.

| Setting in `config.yaml` | Default | Meaning |
|---|---|---|
| `arm.backend` | ros | `ros` (MoveIt), `serial` (arm on this machine's USB, no ROS) or `sim` |
| `ros.arm_group` | arm | MoveIt planning group (`hand` in Waveshare's original config) |
| `ros.max_vel_scale` | 0.5 | MoveIt velocity scaling at 100 % on the operator page |
| `ros.tablet_below_mm` | 30 | tablet box top below the taught touch points |
| `ros.keep_out` | none | extra no-go boxes for MoveIt, e.g. where students stand |
| `arm.port` | auto | serial backend only: serial port, for example `/dev/ttyUSB0` |
| `skills.press_mm` | 3 | how far the stylus presses below the taught height |
| `skills.travel_mm_s` | 50 | top speed in open space (mm/s) at 100 % |
| `skills.near_mm_s` | 15 | top speed near the screen, chocolate and cup, at 100 % |
| `skills.jog_mm_s` | 10 | top speed when jogging, at 100 % |
| `skills.accel_mm_s2` | 100 | how gently moves start and stop (lower = gentler) |
| `skills.gripper_spd` | 300 | gripper speed (servo steps/s; 4096 = one full turn) |
| `skills.hover_mm` | 25 | height above the screen where the slow part of a tap starts |
| `game.robot_move` | tap | `tap` or `point` at start-up (operator can switch live) |
| `game.mode` | easy | difficulty at start-up |
| `game.reward_draw` | true | chocolate for a draw at start-up |
| `game.think_s` | 1.5 | seconds the robot "thinks" on screen before moving |
| `colours` | red, blue, green, yellow | chocolate colours and their screen colours |

**Your calibration** is in the `data/` folder (`poses.yaml`, `tray.yaml`,
`settings.yaml`). Back it up before the event. Code updates never touch it.

**Code**

```
ttt/server.py     web server and game flow
ttt/game.py       game logic: minimax and difficulty modes
ttt/skills.py     arm movements: tap, give chocolate, wave
ttt/ros_arm.py    MoveIt backend: IK, planned moves, tablet box, STOP = cancel goal
ttt/arm.py        direct USB serial backend (+ simulation)
ttt/poses.py      calibration storage, corner interpolation
web/              tablet, screen and operator pages (fonts included, no internet needed)
launch/           laptop launch file (MoveIt + game), joint_limits.example.yaml
tests/            automatic tests: python3 -m pytest tests  and  python3 tests/e2e_sim.py
```
# tic_tac_toe_with_arm
