"""Arm backend that moves the RoArm through ROS 2 + MoveIt.

Same interface as RoArm (arm.py), so skills.py and the game don't change:
    connect, wait_alive, feedback, pose, move_to, gripper, torque,
    estop, reset_estop, stopped, close, workspace, speed_scale

How a move works (like excavation_server): position-only IK via /compute_ik,
then a joint-space goal to /move_action with velocity/acceleration scaling.
MoveIt plans in joint space within the joint limits, and avoids the tablet,
which is added to the planning scene as a box from the taught board corners.

Units: this class speaks mm (like the rest of the app); MoveIt uses metres.
Positions are hand_tcp in the planning frame, read from TF.

The Pi's roarm_driver is open-loop: it forwards MoveIt's joint states to the
servos and never reads them back. So torque-off teaching is disabled here
(MoveIt would not know the arm moved); teach by jogging instead.
"""
from __future__ import annotations

import math
import threading
import time

from .arm import ArmError, Workspace

DEFAULTS = dict(
    arm_group="arm",                 # Waveshare's original config calls it "hand"
    gripper_group="gripper",
    gripper_joint="link3_to_gripper_link",
    ee_link="hand_tcp",
    frame="base_link",
    move_action="/move_action",
    ik_service="/compute_ik",
    scene_service="/apply_planning_scene",
    max_vel_scale=0.5,               # MoveIt velocity scaling at 100 % on the operator page
    max_acc_scale=0.5,
    ref_speed_mm_s=50.0,             # the skills' travel speed that maps to max_vel_scale
    planning_time_s=3.0,
    ik_avoid_collisions=True,
    tablet_collision=True,           # add the tablet as a box to the planning scene
    tablet_below_mm=30.0,            # box top this far below the taught stylus-touch points.
                                     # The arm model reaches ~19 mm below hand_tcp, so this
                                     # needs the stylus tip >= ~15 mm below the gripper jaws.
    tablet_margin_mm=45.0,           # box extends this far beyond the corner cells
    keep_out=[],                     # extra boxes: [{name, min: [x,y,z], max: [x,y,z]}] in mm
    connect_timeout_s=20.0,
    joint_states_topics=["/roarm/joint_states", "/joint_states"],   # gripper position read from any of these
    gripper_step_rad=0.08,           # gripper moves in steps this small (~4.6 deg) instead of
                                     # one jump: a GripperCommand controller snaps otherwise
)


class RosArm:
    sim = False

    def __init__(self, cfg: dict | None = None, verbose: bool = False):
        self.cfg = {**DEFAULTS, **(cfg or {})}
        self.port = "MoveIt"
        self.workspace = Workspace()
        self.speed_scale = 0.5
        self.verbose = verbose
        self._estop = threading.Event()
        self._active_goal = None
        self._goal_lock = threading.Lock()
        self._gripper = None
        self._scene_ids: set[str] = set()   # collision objects we added
        self._node = None
        self._executor = None
        self._spin_thread = None

    # ------------------------------------------------------------ connection
    def connect(self) -> "RosArm":
        try:
            import rclpy
            from rclpy.action import ActionClient
            from rclpy.executors import MultiThreadedExecutor
            from rclpy.node import Node
            import tf2_ros
            from moveit_msgs.action import MoveGroup
            from moveit_msgs.srv import ApplyPlanningScene, GetPositionIK
        except ImportError as e:
            raise ArmError(f"ROS 2 / MoveIt Python packages not found ({e}). "
                           "Source ROS first: source /opt/ros/jazzy/setup.bash")
        if not rclpy.ok():
            rclpy.init()
        c = self.cfg
        self._node = Node("ttt_arm")
        self._tf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf, self._node)
        self._move = ActionClient(self._node, MoveGroup, c["move_action"])
        self._ik = self._node.create_client(GetPositionIK, c["ik_service"])
        self._scene = self._node.create_client(ApplyPlanningScene, c["scene_service"])
        from sensor_msgs.msg import JointState
        topics = c.get("joint_states_topics") or []
        if c.get("joint_states_topic"):          # older config key
            topics = [c["joint_states_topic"]] + list(topics)
        for topic in dict.fromkeys(topics):
            self._node.create_subscription(JointState, topic, self._on_joints, 10)
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

        t = c["connect_timeout_s"]
        if not self._move.wait_for_server(timeout_sec=t):
            raise ArmError(f"MoveIt not found ({c['move_action']} not available after {t:.0f} s). "
                           "Is move_group running, and are both machines on the same ROS_DOMAIN_ID?")
        if not self._ik.wait_for_service(timeout_sec=5.0):
            raise ArmError(f"{c['ik_service']} not available")
        self._log("connected to MoveIt")
        return self

    def close(self) -> None:
        try:
            import rclpy
            if self._executor:
                self._executor.shutdown(timeout_sec=1.0)
            if self._spin_thread:
                self._spin_thread.join(timeout=2.0)
            if self._node:
                self._node.destroy_node()
            rclpy.try_shutdown()
        except Exception:
            pass

    def wait_alive(self, timeout: float = 10.0) -> dict:
        deadline = time.time() + timeout
        while True:
            try:
                return self.feedback()
            except ArmError:
                if time.time() > deadline:
                    raise ArmError(f"No TF from {self.cfg['frame']} to {self.cfg['ee_link']}. "
                                   "Is robot_state_publisher running and joint_states being published?")
                time.sleep(0.3)

    # ------------------------------------------------------------ state
    def feedback(self, timeout: float = 1.0) -> dict:
        from rclpy.time import Time
        try:
            tr = self._tf.lookup_transform(self.cfg["frame"], self.cfg["ee_link"], Time())
        except Exception as e:
            raise ArmError(f"TF lookup failed: {e}")
        p = tr.transform.translation
        out = {"x": p.x * 1000.0, "y": p.y * 1000.0, "z": p.z * 1000.0}
        out["t"] = self._gripper if self._gripper is not None else 0.0
        return out

    def _on_joints(self, msg):
        j = self.cfg["gripper_joint"]
        if j in msg.name:
            self._gripper = float(msg.position[msg.name.index(j)])

    def pose(self) -> dict:
        fb = self.feedback()
        return {k: round(float(fb[k]), 2) for k in ("x", "y", "z", "t")}

    # ------------------------------------------------------------ commands
    def torque(self, on: bool) -> None:
        if not on:
            raise ArmError("Torque off isn't available with MoveIt: the driver doesn't read the "
                           "servos, so MoveIt wouldn't know where you moved the arm and would snap "
                           "it back. Use the jog buttons to teach positions.")

    def move_to(self, x: float, y: float, z: float, speed_mm_s: float = 40.0,
                accel_mm_s2: float = 100.0, tol: float = 6.0, settle_s: float = 3.0,
                scaled: bool = True) -> None:
        self._guard()
        self.workspace.check(x, y, z)
        joints = self._solve_ik(x, y, z)
        c = self.cfg
        k = (self.speed_scale if scaled else 1.0) * min(1.0, speed_mm_s / c["ref_speed_mm_s"])
        vel = max(0.01, min(1.0, c["max_vel_scale"] * k))
        acc = max(0.01, min(1.0, c["max_acc_scale"] * k))
        self._execute(self._joint_constraints(joints, 0.005), c["arm_group"], vel, acc,
                      what=f"({x:.0f}, {y:.0f}, {z:.0f})")

    def gripper(self, rad: float, spd: float = 300, acc: float = 10, wait: bool = True) -> None:
        """rad is the MoveIt gripper joint value (0 = closed, 1.5 = open in the RoArm URDF)."""
        self._guard()
        c = self.cfg
        k = self.speed_scale
        if self._gripper is None:
            raise ArmError("Gripper position unknown: no joint states received on "
                           f"{self.cfg.get('joint_states_topics')}. Check ros.joint_states_topics.")
        start = self._gripper
        n = max(1, math.ceil(abs(rad - start) / c["gripper_step_rad"]))
        for i in range(1, n + 1):
            value = start + (rad - start) * i / n
            self._execute(self._joint_constraints({c["gripper_joint"]: value}, 0.02),
                          c["gripper_group"], max(0.05, c["max_vel_scale"] * k),
                          max(0.05, c["max_acc_scale"] * k), what=f"gripper {value:.2f}")
            self._gripper = value

    # ------------------------------------------------------------ safety
    def estop(self) -> None:
        """Cancel the running MoveIt goal: the controller stops feeding new
        positions and the arm holds where it is. Refuses new moves until reset."""
        self._estop.set()
        with self._goal_lock:
            gh = self._active_goal
        if gh is not None:
            try:
                gh.cancel_goal_async()
            except Exception:
                pass

    def reset_estop(self) -> None:
        self._estop.clear()

    @property
    def stopped(self) -> bool:
        return self._estop.is_set()

    def _guard(self) -> None:
        if self._estop.is_set():
            raise ArmError("E-stop active - call reset_estop() first")

    # ------------------------------------------------------------ planning scene
    def update_scene(self, poses: dict) -> None:
        """Tablet box from the taught corners, plus any keep-out boxes."""
        from moveit_msgs.msg import CollisionObject, PlanningScene
        from moveit_msgs.srv import ApplyPlanningScene

        if not self._scene.wait_for_service(timeout_sec=2.0):
            self._log("planning scene service not available; tablet not added")
            return
        c = self.cfg
        wanted = []
        tablet = tablet_box(poses, c["tablet_below_mm"], c["tablet_margin_mm"])
        if tablet and c["tablet_collision"]:
            wanted.append(self._box("ttt_tablet", *tablet))
        for i, box in enumerate(c["keep_out"] or []):
            lo, hi = box["min"], box["max"]
            centre = [(a + b) / 2 for a, b in zip(lo, hi)]
            size = [abs(b - a) for a, b in zip(lo, hi)]
            wanted.append(self._box(f"ttt_keepout_{box.get('name', i)}", centre, size, (0, 0, 0, 1)))
        keep = {o.id for o in wanted}
        objects = wanted + [self._remove(i) for i in self._scene_ids - keep]
        if not objects:
            return

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = objects
        req = ApplyPlanningScene.Request(scene=scene)
        resp = self._wait(self._scene.call_async(req), 5.0)
        if resp is None or not resp.success:
            self._log("could not update the planning scene")
        else:
            self._scene_ids = keep
            self._log(f"planning scene: {[o.id for o in objects if o.operation == CollisionObject.ADD]}")

    def _box(self, name, centre_mm, size_mm, quat):
        from geometry_msgs.msg import Pose
        from moveit_msgs.msg import CollisionObject
        from shape_msgs.msg import SolidPrimitive
        o = CollisionObject()
        o.id = name
        o.header.frame_id = self.cfg["frame"]
        prim = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[s / 1000.0 for s in size_mm])
        p = Pose()
        p.position.x, p.position.y, p.position.z = (v / 1000.0 for v in centre_mm)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = quat
        o.primitives.append(prim)
        o.primitive_poses.append(p)
        o.operation = CollisionObject.ADD
        return o

    def _remove(self, name):
        from moveit_msgs.msg import CollisionObject
        o = CollisionObject()
        o.id = name
        o.header.frame_id = self.cfg["frame"]
        o.operation = CollisionObject.REMOVE
        return o

    # ------------------------------------------------------------ MoveIt plumbing
    def _solve_ik(self, x, y, z) -> dict:
        from moveit_msgs.srv import GetPositionIK
        c = self.cfg
        req = GetPositionIK.Request()
        ik = req.ik_request
        ik.group_name = c["arm_group"]
        ik.ik_link_name = c["ee_link"]
        ik.avoid_collisions = bool(c["ik_avoid_collisions"])
        ik.pose_stamped.header.frame_id = c["frame"]
        ik.pose_stamped.pose.position.x = x / 1000.0
        ik.pose_stamped.pose.position.y = y / 1000.0
        ik.pose_stamped.pose.position.z = z / 1000.0
        ik.pose_stamped.pose.orientation.w = 1.0   # position-only arm: orientation ignored
        ik.timeout.sec = 1
        resp = self._wait(self._ik.call_async(req), 5.0)
        if resp is None:
            raise ArmError("IK service timed out")
        if resp.error_code.val != 1:
            hint = (" It may be blocked by the tablet box: check the board corners, "
                    "or lower ros.tablet_below_mm." if c["tablet_collision"] else "")
            raise ArmError(f"No arm position reaches ({x:.0f}, {y:.0f}, {z:.0f}) "
                           f"(IK error {resp.error_code.val}).{hint}")
        js = resp.solution.joint_state
        return {n: p for n, p in zip(js.name, js.position) if n != c["gripper_joint"]}

    @staticmethod
    def _joint_constraints(joints: dict, tol: float):
        from moveit_msgs.msg import Constraints, JointConstraint
        cons = Constraints()
        for n, p in joints.items():
            jc = JointConstraint(joint_name=n, position=float(p), tolerance_above=tol,
                                 tolerance_below=tol, weight=1.0)
            cons.joint_constraints.append(jc)
        return cons

    def _execute(self, constraints, group, vel, acc, what=""):
        from moveit_msgs.action import MoveGroup
        from moveit_msgs.msg import MotionPlanRequest
        self._guard()
        req = MotionPlanRequest()
        req.group_name = group
        req.num_planning_attempts = 5
        req.allowed_planning_time = float(self.cfg["planning_time_s"])
        req.max_velocity_scaling_factor = float(vel)
        req.max_acceleration_scaling_factor = float(acc)
        req.goal_constraints.append(constraints)
        req.start_state.is_diff = True          # start from MoveIt's current state
        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        goal.planning_options.plan_only = False

        gh = self._wait(self._move.send_goal_async(goal), 10.0)
        if gh is None or not gh.accepted:
            raise ArmError(f"MoveIt rejected the move to {what}")
        with self._goal_lock:
            self._active_goal = gh
        try:
            if self._estop.is_set():
                gh.cancel_goal_async()
            res = self._wait(gh.get_result_async(), 120.0)
        finally:
            with self._goal_lock:
                self._active_goal = None
        if self._estop.is_set():
            raise ArmError("E-stop active")
        if res is None:
            raise ArmError(f"MoveIt timed out moving to {what}")
        code = res.result.error_code.val
        if code != 1:
            raise ArmError(f"MoveIt could not move to {what} (error {code}: {MOVEIT_ERRORS.get(code, '')})")

    @staticmethod
    def _wait(future, timeout):
        end = time.time() + timeout
        while not future.done():
            if time.time() > end:
                return None
            time.sleep(0.01)
        return future.result()

    def _log(self, msg):
        print(f"[arm] {msg}", flush=True)


MOVEIT_ERRORS = {
    -1: "planning failed", -2: "invalid motion plan", -4: "control failed",
    -5: "unable to aquire sensor data", -6: "timed out", -7: "preempted (stopped)",
    -10: "start state in collision", -12: "goal in collision", -21: "frame not found",
    -31: "no IK solution",
}


# ------------------------------------------------------------------ geometry
def tablet_box(poses: dict, below_mm: float, margin_mm: float, thickness_mm: float = 10.0):
    """Box (centre mm, size mm, quaternion) under the four taught corner cells.

    The corners are where the stylus touches, so the screen is just below
    them; the box top sits `below_mm` lower, tilted with the tablet."""
    c = poses.get("board_corners") or {}
    if not all(k in c for k in (1, 3, 7, 9)):
        return None
    p1, p3, p7, p9 = (c[k] for k in (1, 3, 7, 9))
    u = [((p3[i] - p1[i]) + (p9[i] - p7[i])) / 2 for i in range(3)]   # along a row
    v = [((p7[i] - p1[i]) + (p9[i] - p3[i])) / 2 for i in range(3)]   # down a column
    lu, lv = _norm(u), _norm(v)
    if lu < 10 or lv < 10:
        return None
    u = [a / lu for a in u]
    n = _cross(u, v)
    n = [a / _norm(n) for a in n]
    if n[2] < 0:
        n = [-a for a in n]
    v = _cross(n, u)                                   # exact right-handed basis u, v, n
    centre = [(p1[i] + p3[i] + p7[i] + p9[i]) / 4 for i in range(3)]
    centre = [centre[i] - n[i] * (below_mm + thickness_mm / 2) for i in range(3)]
    size = [lu + 2 * margin_mm, lv + 2 * margin_mm, thickness_mm]
    return centre, size, _quat_from_axes(u, v, n)


def _norm(a):
    return math.sqrt(sum(x * x for x in a))


def _cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]


def _quat_from_axes(x, y, z):
    """Rotation whose columns are the unit axes x, y, z -> quaternion (x, y, z, w)."""
    m00, m01, m02 = x[0], y[0], z[0]
    m10, m11, m12 = x[1], y[1], z[1]
    m20, m21, m22 = x[2], y[2], z[2]
    tr = m00 + m11 + m22
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return ((m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s)
    if m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2
        return (0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s)
    if m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2
        return ((m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s)
    s = math.sqrt(1.0 + m22 - m00 - m11) * 2
    return ((m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s)
