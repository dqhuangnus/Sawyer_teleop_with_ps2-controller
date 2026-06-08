#!/usr/bin/env python3
"""
ps2_sawyer_teleop_viz.py — Generic USB gamepad teleop for Sawyer + RViz
========================================================================
Confirmed button/axis indices from your controller:
  A=0  B=1  X=2  Y=3  L1=4  L2=5  Select=6  Start=7  Mode=8
  Left stick click=9  Right stick click=10

CONTROL SCHEME
══════════════
  POSITION / ORIENTATION  (hold L1 or L2 to enable motion)
    Left  stick  fwd/back   → EE +X / -X
    Left  stick  left/right → EE +Y / -Y

    Hold L1 (button 4) — DEFAULT mode (yaw + Z):
      Right stick  fwd/back   → EE +Z / -Z  (up/down)
      Right stick  left/right → yaw

    Hold L2 (button 5) — ROLL+PITCH mode (fine orientation):
      Right stick  left/right → roll
      Right stick  fwd/back   → pitch
      (Z is frozen in this mode)

  GRIPPER
    Y  (button 3)   → toggle open / close

  HOMING
    A  (button 0)   → startup: move to HOME (safe interpolation)
    B  (button 1)   → startup: skip home, start from current position
    Start (button 7)→ mid-session: return to HOME safely
    Select(button 6)→ mid-session: re-anchor IK at current position

  ORIENTATION HELPERS
    X  (button 2)   → reset orientation to home (clears accumulated drift)

  KEYBOARD
    r               → same as Start (return to HOME)
"""

# libraries
import sys
import os
import math
import importlib.util
import intera_interface                          # Sawyer SDK
import rospy
import tf2_ros
import tf.transformations as tft
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
import pygame

# Data-collection helpers (live alongside this script in scripts/).
# Degrade gracefully if capture deps (pypylon / websocket-client / h5py) absent.
try:
    from sensors import TactileReader, BaslerCameraManager
    from data_recorder import DataRecorder
    _RECORDING_AVAILABLE = True
except Exception:
    _RECORDING_AVAILABLE = False

# ─── relaxedIK solver from directory ──────────────────────────────────────
_RIK_ROOT     = '/root/catkin_ws/src/relaxed_ik_core'       # repository
_RIK_WRAPPER  = _RIK_ROOT + '/wrappers/python_wrapper.py'   # Python bindings
_RIK_SETTINGS = _RIK_ROOT + '/configs/settings.yaml'        # robot config

_spec = importlib.util.spec_from_file_location("python_wrapper", _RIK_WRAPPER)
_mod  = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_mod)                   # dynamically import wrapper
RelaxedIKRust = _mod.RelaxedIKRust               # IK solver class

# ─── robot joint names ─────────────────────────────────────────────────────
JOINT_NAMES = [
    'right_j0', 'right_j1', 'right_j2',
    'right_j3', 'right_j4', 'right_j5', 'right_j6'
]
# gripper finger joints (robotiq 2F-85 mimic chain)
GRIPPER_JOINT_NAMES = [
    'finger_joint',
    'right_outer_knuckle_joint',
    'left_inner_knuckle_joint',  'right_inner_knuckle_joint',
    'left_inner_finger_joint',   'right_inner_finger_joint',
]
GRIPPER_OPEN        = 0.0   # fully open finger position (rad)
GRIPPER_CLOSE       = 0.8   # fully closed finger position (rad)

DEFAULT_HOME_CONFIG = [0.2984, -1.2459, -0.6619, 1.3025, 0.2306, 1.5009, -0.2057]

# ─── when moving sawyer to home position, the settings ────────────────────
SAFE_HOME_SPEED       = 0.15   # 0–1 joint speed
SAFE_HOME_TIMEOUT     = 15.0
SAFE_HOME_STEPS       = 8      # steps to take
SAFE_HOME_STEP_THRESH = 0.01   # rad — joint is close to home then skip
J6_MAX_DELTA_PER_STEP = 0.40   # max rotation wrist joint

# ─── TF frame names ────────────────────────────────────────────────────────
BASE_FRAME  = 'reference/base'       # robot base frame
EE_FRAME    = 'reference/right_hand' # end-effector frame
TOOL_LENGTH = 0.212                  # fingertip offset from wrist (metres)

# ─── Sensor bowl RViz dimensions ───────────────────────────────────────────
BOWL_OUTER = 0.80    # outer rim diameter (m)
BOWL_BASE  = 0.24    # flat inner base size (m)
BOWL_RISE  = 0.09    # slope rise height (m)
BOWL_THICK = 0.005   # wall/floor thickness (m)
BRKT_TALL  = 0.09    # bracket tall side (m)
BRKT_SHORT = 0.05    # bracket short side (m)

# ─── controller button indices (confirmed for this gamepad) ────────────────
BTN_A      = 0    # startup: go to HOME
BTN_B      = 1    # startup: skip home
BTN_X      = 2    # reset orientation to home
BTN_Y      = 3    # toggle gripper
BTN_L1     = 4    # hold → enable motion, mode 0 (yaw + Z)
BTN_L2     = 5    # hold → enable motion, mode 1 (roll + pitch)
BTN_SELECT = 6    # re-anchor IK
BTN_START  = 7    # return to HOME
BTN_MODE   = 8    # (spare)
BTN_LS     = 9    # left stick click  (spare)
BTN_RS     = 10   # right stick click (spare)

# ─── controller axis indices ───────────────────────────────────────────────
AXIS_LX = 0   # left stick horizontal  → robot Y
AXIS_LY = 1   # left stick vertical    → robot X  (inverted below)
AXIS_RX = 3   # right stick horizontal → yaw / roll
AXIS_RY = 4   # right stick vertical   → Z / pitch (inverted below)

# DEADZONE — only one threshold needed; STICK_MIN_TOTAL removed (redundant)
STICK_DEADZONE = 0.12   # ignore stick values below this (per-axis)

# ─── velocity scales — how fast the EE moves at full stick deflection ──────
# Kept moderate — proximity scaling handles the sensitive near-base region
XY_VEL_SCALE    = 0.20   # m/s
Z_VEL_SCALE     = 0.14   # m/s
YAW_VEL_SCALE   = 0.45   # rad/s
ROLL_VEL_SCALE  = 0.35   # rad/s
PITCH_VEL_SCALE = 0.35   # rad/s

# ─── IK smoothing — ONE layer only, applied to joint commands ─────────────
# The previous code had 4 stacked damping layers (goal EMA + cartesian clamp
# + joint rate-limit + joint EMA) which caused quantised micro-steps and
# audible motor buzzing.  We keep only the joint-space EMA + rate-limit.
JOINT_SMOOTH_ALPHA = 0.35   # EMA weight for new IK solution (0=frozen, 1=raw)
MAX_JOINT_DELTA    = 0.08   # max rad any joint can jump per cycle
JOINT_CMD_DEADBAND = 0.0005 # rad — skip robot command if no joint changed more
                             # than this. Kept tiny so slow stick motion isn't
                             # quantised into discrete steps.

# ─── Sawyer joint limits (rad) ─────────────────────────────────────────────
JOINT_LIMITS = [
    (-3.0503,  3.0503),  # right_j0
    (-3.8095,  2.2736),  # right_j1
    (-3.0426,  3.0426),  # right_j2
    (-3.0439,  3.0439),  # right_j3
    (-2.9761,  2.9761),  # right_j4
    (-2.9761,  2.9761),  # right_j5
    (-4.7124,  4.7124),  # right_j6
]

# ─── Smooth homing parameters ──────────────────────────────────────────────
HOME_RATE_HZ          = 50     # Hz for homing interpolation loop
HOME_DURATION_PER_RAD = 2.5    # seconds per radian of largest joint travel
HOME_MIN_DURATION     = 3.0    # minimum homing duration (s)
HOME_MAX_DURATION     = 10.0   # maximum homing duration (s)


# ─── RViz marker helper functions ──────────────────────────────────────────

def _sphere(ns, mid, r, g, b, size=0.025):
    """Return a sphere Marker (used for goal/actual EE position dots)."""
    m = Marker()
    m.header.frame_id = BASE_FRAME
    m.ns, m.id = ns, mid
    m.type   = Marker.SPHERE
    m.action = Marker.ADD
    m.scale.x = m.scale.y = m.scale.z = size
    m.color   = ColorRGBA(r, g, b, 1.0)
    m.pose.orientation.w = 1.0
    return m

def _text(ns, mid):
    """Return a text Marker (used for the HUD overlay in RViz)."""
    m = Marker()
    m.header.frame_id = BASE_FRAME
    m.ns, m.id = ns, mid
    m.type   = Marker.TEXT_VIEW_FACING
    m.action = Marker.ADD
    m.scale.z = 0.03
    m.color   = ColorRGBA(1.0, 1.0, 1.0, 1.0)
    m.pose.orientation.w = 1.0
    m.pose.position.x, m.pose.position.z = 0.4, 0.65
    return m

def _build_sensor_bowl_markers(cx, cy, cz):
    """Build a list of Markers that draw the sensor bowl in RViz."""
    markers = []
    uid = 100
    GREY = ColorRGBA(0.45, 0.45, 0.50, 0.90)
    WOOD = ColorRGBA(0.87, 0.80, 0.60, 1.00)
    half_o = BOWL_OUTER / 2
    half_b = BOWL_BASE  / 2
    wt     = BOWL_THICK

    def cube(lx, ly, lz, sx, sy, sz, color, ns):
        nonlocal uid
        m = Marker()
        m.header.frame_id    = BASE_FRAME
        m.ns, m.id           = ns, uid;  uid += 1
        m.type               = Marker.CUBE
        m.action             = Marker.ADD
        m.pose.position.x    = float(cx + lx)
        m.pose.position.y    = float(cy + ly)
        m.pose.position.z    = float(cz + lz)
        m.pose.orientation.w = 1.0
        m.scale.x, m.scale.y, m.scale.z = float(sx), float(sy), float(sz)
        m.color = color
        return m

    def tri_list(tris, color, ns):
        nonlocal uid
        m = Marker()
        m.header.frame_id    = BASE_FRAME
        m.ns, m.id           = ns, uid;  uid += 1
        m.type               = Marker.TRIANGLE_LIST
        m.action             = Marker.ADD
        m.pose.position.x    = float(cx)
        m.pose.position.y    = float(cy)
        m.pose.position.z    = float(cz)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 1.0
        m.color = color
        for tri in tris:
            for v in tri:
                p = Point()
                p.x, p.y, p.z = float(v[0]), float(v[1]), float(v[2])
                m.points.append(p)
        return m

    markers.append(cube(0, 0, wt/2, BOWL_BASE, BOWL_BASE, wt, GREY, "bowl_base"))

    slope_defs = [
        dict(il=(-half_b,  half_b, 0), ir=( half_b,  half_b, 0),
             ol=(-half_o,  half_o, BOWL_RISE), or_=( half_o,  half_o, BOWL_RISE)),
        dict(il=(-half_b, -half_b, 0), ir=( half_b, -half_b, 0),
             ol=(-half_o, -half_o, BOWL_RISE), or_=( half_o, -half_o, BOWL_RISE)),
        dict(il=(-half_b, -half_b, 0), ir=(-half_b,  half_b, 0),
             ol=(-half_o, -half_o, BOWL_RISE), or_=(-half_o,  half_o, BOWL_RISE)),
        dict(il=( half_b, -half_b, 0), ir=( half_b,  half_b, 0),
             ol=( half_o, -half_o, BOWL_RISE), or_=( half_o,  half_o, BOWL_RISE)),
    ]
    tris = []
    for d in slope_defs:
        il, ir, ol, or_ = d['il'], d['ir'], d['ol'], d['or_']
        tris.append((il, ir, ol))
        tris.append((ir, or_, ol))
    markers.append(tri_list(tris, GREY, "bowl_slopes"))

    rim_lip_z = BOWL_RISE + wt / 2
    for cfg in [
        dict(lx=0,       ly= half_o, lz=rim_lip_z, sx=BOWL_OUTER, sy=wt, sz=wt),
        dict(lx=0,       ly=-half_o, lz=rim_lip_z, sx=BOWL_OUTER, sy=wt, sz=wt),
        dict(lx=-half_o, ly=0,       lz=rim_lip_z, sx=wt, sy=BOWL_OUTER, sz=wt),
        dict(lx= half_o, ly=0,       lz=rim_lip_z, sx=wt, sy=BOWL_OUTER, sz=wt),
    ]:
        markers.append(cube(cfg['lx'], cfg['ly'], cfg['lz'],
                            cfg['sx'], cfg['sy'], cfg['sz'], GREY, "bowl_rim"))

    brkt_tris = []
    for (bx, by, idx, _) in [
        ( half_o,  half_o, -1, 0),
        (-half_o,  half_o, +1, 0),
        ( half_o, -half_o, -1, 0),
        (-half_o, -half_o, +1, 0),
    ]:
        brkt_tris.append(((bx, by, 0.0), (bx, by, BRKT_TALL),
                          (bx + idx*BRKT_SHORT, by, 0.0)))
    markers.append(tri_list(brkt_tris, WOOD, "bowl_brackets"))
    return markers


# ─── Main teleop node ──────────────────────────────────────────────────────
class PS2TeleopVizNode:
    def __init__(self):
        rospy.init_node('ps2_sawyer_teleop_viz', anonymous=False)

        # ── ROS parameters ───────────────────────────────────────────────
        self.control_rate     = rospy.get_param('~control_rate',     50.0)
        self.workspace_centre = rospy.get_param('~workspace_centre', [0.70, 0.0, 0.20])

        _sbp = rospy.get_param('~sensor_bowl_pos', [0.70, 0.0, 0.05])
        if isinstance(_sbp, str):
            import ast; _sbp = ast.literal_eval(_sbp)
        self._sensor_bowl_pos = [float(v) for v in _sbp]

        self.HOME_CONFIG = list(DEFAULT_HOME_CONFIG)

        # ── Gamepad init ─────────────────────────────────────────────────
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            rospy.logfatal("[ctrl] No joystick detected!")
            sys.exit(1)
        self._joy = pygame.joystick.Joystick(0)
        self._joy.init()
        rospy.loginfo("[ctrl] %s  axes=%d  buttons=%d",
                      self._joy.get_name(),
                      self._joy.get_numaxes(),
                      self._joy.get_numbuttons())
        self._prev_btn = {}

        # ── TF listener ──────────────────────────────────────────────────
        self._tf_buf = tf2_ros.Buffer()
        self._tf_lis = tf2_ros.TransformListener(self._tf_buf)

        # ── Joint state publisher ─────────────────────────────────────────
        self._js_pub = rospy.Publisher('/joint_states', JointState, queue_size=5)
        self._current_angles = list(self.HOME_CONFIG)

        # ── RelaxedIK solver ─────────────────────────────────────────────
        rospy.loginfo("[ctrl] Loading RelaxedIK...")
        saved = os.getcwd()
        os.chdir(_RIK_ROOT)
        try:
            self.rik = RelaxedIKRust(setting_file_path=_RIK_SETTINGS)
        finally:
            os.chdir(saved)
        rospy.loginfo("[ctrl] RelaxedIK OK")

        # ── Enable Sawyer safety system ──────────────────────────────────
        rs = intera_interface.RobotEnable(intera_interface.CHECK_VERSION)
        rs.enable()
        rospy.loginfo("[ctrl] Robot enabled")

        self._limb = intera_interface.Limb('right')

        # ── Robotiq gripper init ─────────────────────────────────────────
        try:
            from pyrobotiqgripper import RobotiqGripper
            self._gripper       = RobotiqGripper()
            self._gripper.activate()
            rospy.sleep(0.5)
            self._gripper_ready = True
            rospy.loginfo("[ctrl] Gripper activated")
        except Exception as e:
            self._gripper_ready = False
            rospy.logwarn("[ctrl] Gripper init failed: %s", e)
        self._gripper_open = False

        # ── RViz marker setup ────────────────────────────────────────────
        self._mk_pub    = rospy.Publisher('/teleop_viz', Marker, queue_size=20)
        self._mk_goal   = _sphere("goal",   0, 0.0, 1.0, 0.0, 0.03)
        self._mk_actual = _sphere("actual", 1, 1.0, 0.0, 0.0, 0.03)
        self._mk_info   = _text("info", 4)

        self._mk_box = Marker()
        self._mk_box.header.frame_id = BASE_FRAME
        self._mk_box.ns, self._mk_box.id = "virtual_box_fill", 10
        self._mk_box.type   = Marker.CUBE
        self._mk_box.action = Marker.ADD
        self._mk_box.scale.x = 0.80
        self._mk_box.scale.y = 0.80
        self._mk_box.scale.z = 0.40
        self._mk_box.color   = ColorRGBA(1.0, 0.5, 0.0, 0.15)
        self._mk_box.pose.orientation.w = 1.0

        self._mk_box_edges = Marker()
        self._mk_box_edges.header.frame_id = BASE_FRAME
        self._mk_box_edges.ns, self._mk_box_edges.id = "virtual_box_edges", 11
        self._mk_box_edges.type   = Marker.LINE_LIST
        self._mk_box_edges.action = Marker.ADD
        self._mk_box_edges.scale.x = 0.005
        self._mk_box_edges.color   = ColorRGBA(1.0, 0.0, 0.0, 1.0)
        self._mk_box_edges.pose.orientation.w = 1.0

        self._mk_table = Marker()
        self._mk_table.header.frame_id = BASE_FRAME
        self._mk_table.ns, self._mk_table.id = "table", 20
        self._mk_table.type   = Marker.CUBE
        self._mk_table.action = Marker.ADD
        self._mk_table.scale.x = 1.80
        self._mk_table.scale.y = 1.20
        self._mk_table.scale.z = 0.05
        self._mk_table.color   = ColorRGBA(0.55, 0.40, 0.25, 1.0)
        self._mk_table.pose.orientation.w = 1.0

        self._mk_legs = []
        for i in range(4):
            leg = Marker()
            leg.header.frame_id = BASE_FRAME
            leg.ns, leg.id = "table_leg", 30 + i
            leg.type   = Marker.CYLINDER
            leg.action = Marker.ADD
            leg.scale.x = leg.scale.y = 0.06
            leg.scale.z = 0.80
            leg.color   = ColorRGBA(0.55, 0.40, 0.25, 1.0)
            leg.pose.orientation.w = 1.0
            self._mk_legs.append(leg)

        self._mk_wood_box = Marker()
        self._mk_wood_box.header.frame_id = BASE_FRAME
        self._mk_wood_box.ns, self._mk_wood_box.id = "wood_box", 40
        self._mk_wood_box.type   = Marker.CUBE
        self._mk_wood_box.action = Marker.ADD
        self._mk_wood_box.scale.x = 0.24
        self._mk_wood_box.scale.y = 0.24
        self._mk_wood_box.scale.z = 0.05
        self._mk_wood_box.color   = ColorRGBA(0.82, 0.65, 0.40, 1.0)
        self._mk_wood_box.pose.orientation.w = 1.0

        # ── Teleop runtime state ─────────────────────────────────────────
        self._enabled          = False
        self._current_goal     = None
        self._home_pos         = None
        self._home_quat        = None
        self._cmd_angles       = list(self.HOME_CONFIG)
        self._last_sent_angles = list(self.HOME_CONFIG)   # deadband comparison
        self._viz_tick         = 0

        self._ori_roll  = 0.0
        self._ori_pitch = 0.0
        self._ori_yaw   = 0.0
        self._ori_mode  = 0   # 0 = yaw+Z (L1),  1 = roll+pitch (L2)

        # ── Data collection (Basler ×3 + uSkin tactile) ───────────────────
        self.record_enabled   = rospy.get_param('~record_enabled', True)
        self.record_rate      = rospy.get_param('~record_rate', 20.0)
        self.save_dir         = rospy.get_param('~save_dir', '/root/collected_data')
        self.xela_ws_url      = rospy.get_param('~xela_ws_url', 'ws://localhost:5000')
        self.n_taxels         = rospy.get_param('~tactile_taxels', 24)
        self.tac_hist         = rospy.get_param('~tactile_history', 5)
        self.camera_ips       = rospy.get_param('~camera_ips', {
            'image_left':  '192.168.1.130',
            'image_right': '192.168.1.120',
            'image_top':   '192.168.1.100'})
        self.cam_scale        = rospy.get_param('~camera_scale',   0.5)
        self.cam_binning      = rospy.get_param('~camera_binning', 2)
        self.cam_fps          = rospy.get_param('~camera_fps',     10)
        self.record_realsense = rospy.get_param('~record_realsense', False)

        self.recorder  = None
        self.recording = False
        self.ep_count  = 0
        self._init_recording()

    # ── Data collection: start sensors + build recorder ───────────────────
    def _init_recording(self):
        if not self.record_enabled:
            rospy.loginfo("[ctrl] Recording disabled (record_enabled=false)")
            return
        if not _RECORDING_AVAILABLE:
            rospy.logwarn("[ctrl] Recording deps missing (pypylon/websocket/h5py) — disabled")
            return

        self.tactile = None
        try:
            self.tactile = TactileReader(ws_url=self.xela_ws_url,
                                         n_per_finger=self.n_taxels,
                                         history_len=self.tac_hist)
            self.tactile.start()
            rospy.loginfo("[ctrl] Tactile reader started (%s)", self.xela_ws_url)
        except Exception as e:
            rospy.logwarn("[ctrl] Tactile unavailable: %s", e)

        self.basler = None
        try:
            self.basler = BaslerCameraManager(self.camera_ips, scale=self.cam_scale,
                                              binning=self.cam_binning, fps=self.cam_fps)
            self.basler.start_bg()
            rospy.loginfo("[ctrl] Basler cameras: %s", self.basler.names)
        except Exception as e:
            rospy.logwarn("[ctrl] Basler cameras unavailable: %s", e)

        self.realsense = None
        if self.record_realsense:
            try:
                from realsense_camera import RealSenseCamera
                self.realsense = RealSenseCamera()
                self.realsense.start_bg()
                rospy.loginfo("[ctrl] RealSense started")
            except Exception as e:
                rospy.logwarn("[ctrl] RealSense unavailable: %s", e)

        gripper = self._gripper if getattr(self, '_gripper_ready', False) else None
        self.recorder = DataRecorder(
            limb=self._limb, gripper=gripper, tactile=self.tactile,
            basler=self.basler, realsense=self.realsense,
            rate_hz=self.record_rate, save_dir=self.save_dir)
        rospy.loginfo("[ctrl] Recorder ready -> %s  (r=record  f=finish+save  d=discard)",
                      self.save_dir)

    def _handle_record_key(self, key):
        if self.recorder is None:
            rospy.logwarn_throttle(2.0, "[ctrl] recording not available")
            return
        if key == 'r' and not self.recording:
            self.recorder.start()
            self.recording = True
            rospy.loginfo("[ctrl] >>> RECORDING episode")
        elif key == 'f' and self.recording:
            self.recorder.stop()
            self.recording = False
            path = self.recorder.save(tag="ep%03d" % self.ep_count)
            if path:
                self.ep_count += 1
            rospy.loginfo("[ctrl] <<< saved %d frames", len(self.recorder))
        elif key == 'd' and self.recording:
            self.recorder.stop()
            self.recording = False
            rospy.loginfo("[ctrl] episode discarded")

    def _shutdown_recording(self):
        if self.recording and self.recorder is not None:
            self.recorder.stop()
        for obj in (getattr(self, 'tactile', None), getattr(self, 'basler', None),
                    getattr(self, 'realsense', None)):
            try:
                if obj is not None:
                    obj.stop()
            except Exception:
                pass

    # ─── Low-level input helpers ───────────────────────────────────────────

    def _axis(self, idx):
        """Read a stick axis and zero it if inside the deadzone."""
        v = self._joy.get_axis(idx)
        return v if abs(v) > STICK_DEADZONE else 0.0

    def _btn_pressed(self, idx):
        """Return True only on the rising edge (button just pressed)."""
        cur  = bool(self._joy.get_button(idx))
        prev = self._prev_btn.get(idx, False)
        self._prev_btn[idx] = cur
        return cur and not prev

    def _btn_held(self, idx):
        """Return True as long as the button is physically held down."""
        return bool(self._joy.get_button(idx))

    # ─── Proximity scaling — only anti-jitter helper kept ─────────────────
    # The previous version had four stacked damping layers:
    #   1. CARTESIAN_MAX_DELTA (goal clamp)
    #   2. POSITION_SMOOTH_ALPHA (goal EMA)
    #   3. MAX_JOINT_DELTA (joint rate-limit)
    #   4. JOINT_SMOOTH_ALPHA (joint EMA)
    # Layers 1 and 2 caused quantised micro-steps and audible motor buzzing
    # because the goal barely moved each cycle, so IK produced tiny repeated
    # joint adjustments that the motors tried (and failed) to track smoothly.
    # We keep only layers 3 and 4 (joint-space), which is the right place to
    # filter because it directly controls what the servo controllers see.
    # Proximity scaling is kept because it addresses the actual root cause
    # (singularity amplification near the base) rather than masking it.

    PROXIMITY_RADIUS = 0.35   # m from home where XY motion is scaled down
    PROXIMITY_SCALE  = 0.75   # multiplier applied to XY delta near base

    def _apply_proximity_scale(self, new_gx, new_gy):
        """Scale down XY delta when close to home to reduce singularity amplification."""
        if self._home_pos is None:
            return new_gx, new_gy
        hx, hy, _ = self._home_pos
        dist = math.hypot(self._current_goal[0] - hx, self._current_goal[1] - hy)
        if dist < self.PROXIMITY_RADIUS:
            dx = (new_gx - self._current_goal[0]) * self.PROXIMITY_SCALE
            dy = (new_gy - self._current_goal[1]) * self.PROXIMITY_SCALE
            return self._current_goal[0] + dx, self._current_goal[1] + dy
        return new_gx, new_gy

    # ─── Joint / robot publishing ──────────────────────────────────────────

    def _publish_joint_states(self, angles, command_robot=True):
        """Publish joint angles to /joint_states (RViz) and optionally to hardware."""
        v = GRIPPER_OPEN if self._gripper_open else GRIPPER_CLOSE
        gripper_pos = [v, v, v, v, -v, -v]
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name     = JOINT_NAMES + GRIPPER_JOINT_NAMES
        msg.position = list(angles) + gripper_pos
        msg.velocity = [0.0] * (7 + len(GRIPPER_JOINT_NAMES))
        msg.effort   = [0.0] * (7 + len(GRIPPER_JOINT_NAMES))
        self._js_pub.publish(msg)
        if command_robot:
            try:
                self._limb.set_joint_positions(dict(zip(JOINT_NAMES, angles)))
            except Exception as e:
                rospy.logwarn("[hw] set_joint_positions failed: %s", e)
        self._current_angles = list(angles)

    def _lookup_ee(self):
        """Look up EE transform from TF. Returns (pos, quat) or (None, None)."""
        try:
            t  = self._tf_buf.lookup_transform(
                BASE_FRAME, EE_FRAME, rospy.Time(0), rospy.Duration(0.0))
            tr = t.transform.translation
            ro = t.transform.rotation
            return [tr.x, tr.y, tr.z], [ro.x, ro.y, ro.z, ro.w]
        except Exception:
            return None, None

    # ─── Homing ────────────────────────────────────────────────────────────

    def _move_to_home(self):
        """Smoothly drive all joints to HOME using cosine-eased set_joint_positions."""
        rospy.loginfo("[ctrl] Reading current joints for homing...")
        cur_dict = self._limb.joint_angles()
        current  = [cur_dict.get(n, self.HOME_CONFIG[i]) for i, n in enumerate(JOINT_NAMES)]

        rospy.loginfo("[ctrl] Current: %s", [f"{a:.3f}" for a in current])
        rospy.loginfo("[ctrl] Target:  %s", [f"{a:.3f}" for a in self.HOME_CONFIG])

        max_delta = max(abs(self.HOME_CONFIG[i] - current[i]) for i in range(7))
        duration  = max(HOME_MIN_DURATION,
                        min(HOME_MAX_DURATION, max_delta * HOME_DURATION_PER_RAD))
        n_steps   = int(duration * HOME_RATE_HZ)
        rate      = rospy.Rate(HOME_RATE_HZ)

        rospy.loginfo("[ctrl] Homing: %.1f s, %d steps (max delta=%.3f rad)",
                      duration, n_steps, max_delta)

        for step in range(1, n_steps + 1):
            if rospy.is_shutdown():
                return
            t    = step / n_steps
            ease = 0.5 - 0.5 * math.cos(math.pi * t)   # cosine ease-in-out
            waypoint = {name: current[i] + ease * (self.HOME_CONFIG[i] - current[i])
                        for i, name in enumerate(JOINT_NAMES)}
            self._limb.set_joint_positions(waypoint)
            rate.sleep()

        rospy.loginfo("[ctrl] HOME reached")

    def _reset_orientation_to_home(self):
        """Snap the integrated roll/pitch/yaw back to the home orientation."""
        if self._home_quat is not None:
            r, p, y = tft.euler_from_quaternion(self._home_quat)
            self._ori_roll  = r
            self._ori_pitch = p
            self._ori_yaw   = y
            rospy.loginfo("[ctrl] Orientation reset to home (r=%.3f p=%.3f y=%.3f)", r, p, y)

    # ─── RViz marker publishers ────────────────────────────────────────────

    def _publish_box(self):
        """Publish the transparent workspace boundary box and red wireframe edges."""
        hx, hy, _ = self._home_pos
        x0, x1 = hx - 0.40, hx + 0.40
        y0, y1 = hy - 0.40, hy + 0.40
        z0 = float(self._sensor_bowl_pos[2])
        z1 = z0 + 0.40

        self._mk_box.header.stamp = rospy.Time.now()
        self._mk_box.pose.position.x = hx
        self._mk_box.pose.position.y = hy
        self._mk_box.pose.position.z = z0 + 0.20
        self._mk_pub.publish(self._mk_box)

        c = [(x0,y0,z0),(x1,y0,z0),(x1,y1,z0),(x0,y1,z0),
             (x0,y0,z1),(x1,y0,z1),(x1,y1,z1),(x0,y1,z1)]
        edges = [(0,1),(1,2),(2,3),(3,0),(0,4),(1,5),(2,6),(3,7)]
        self._mk_box_edges.header.stamp = rospy.Time.now()
        self._mk_box_edges.points = []
        for a, b in edges:
            self._mk_box_edges.points.append(Point(*c[a]))
            self._mk_box_edges.points.append(Point(*c[b]))
        self._mk_pub.publish(self._mk_box_edges)

    def _publish_sensor_bowl(self):
        """Publish all sensor bowl markers."""
        now = rospy.Time.now()
        hx, hy, _ = self._home_pos
        cz = float(self._sensor_bowl_pos[2])
        for m in _build_sensor_bowl_markers(hx, hy, cz):
            m.header.stamp = now
            self._mk_pub.publish(m)

    def _publish_table(self):
        """Publish the table surface, four legs, and wooden sensor platform."""
        hx, hy, _ = self._home_pos
        now = rospy.Time.now()
        table_top_z = 0.0
        thickness   = 0.05

        self._mk_table.header.stamp = now
        self._mk_table.pose.position.x = hx
        self._mk_table.pose.position.y = hy
        self._mk_table.pose.position.z = table_top_z - thickness / 2.0
        self._mk_pub.publish(self._mk_table)

        leg_z = table_top_z - thickness - 0.40
        for leg, (ox, oy) in zip(self._mk_legs,
                                  [(0.80,0.55),(0.80,-0.55),(-0.80,0.55),(-0.80,-0.55)]):
            leg.header.stamp = now
            leg.pose.position.x = hx + ox
            leg.pose.position.y = hy + oy
            leg.pose.position.z = leg_z
            self._mk_pub.publish(leg)

        self._mk_wood_box.header.stamp = now
        self._mk_wood_box.pose.position.x = hx
        self._mk_wood_box.pose.position.y = hy
        self._mk_wood_box.pose.position.z = table_top_z + self._mk_wood_box.scale.z / 2.0
        self._mk_pub.publish(self._mk_wood_box)

    def _publish_markers(self, goal_pos, actual_pos, enabled):
        """Publish goal sphere, actual EE sphere, and HUD text overlay."""
        now = rospy.Time.now()
        mode_str = "L1:YAW+Z" if self._ori_mode == 0 else "L2:ROLL+PITCH"

        if goal_pos:
            self._mk_goal.header.stamp = now
            self._mk_goal.pose.position.x = goal_pos[0]
            self._mk_goal.pose.position.y = goal_pos[1]
            self._mk_goal.pose.position.z = goal_pos[2]
            self._mk_pub.publish(self._mk_goal)

        if actual_pos:
            self._mk_actual.header.stamp = now
            self._mk_actual.pose.position.x = actual_pos[0]
            self._mk_actual.pose.position.y = actual_pos[1]
            self._mk_actual.pose.position.z = actual_pos[2] - TOOL_LENGTH
            self._mk_pub.publish(self._mk_actual)

        if goal_pos and actual_pos:
            err = math.sqrt(sum((a-b)**2 for a,b in zip(goal_pos, actual_pos)))
            self._mk_info.header.stamp = now
            self._mk_info.text = (
                f"{'ENABLED' if enabled else 'DISABLED'}  [{mode_str}]\n"
                f"Goal:   [{goal_pos[0]:.3f}, {goal_pos[1]:.3f}, {goal_pos[2]:.3f}]\n"
                f"Actual: [{actual_pos[0]:.3f}, {actual_pos[1]:.3f}, {actual_pos[2]:.3f}]\n"
                f"Error:  {err*1000:.1f} mm\n"
                f"RPY:    [{math.degrees(self._ori_roll):.1f}°, "
                f"{math.degrees(self._ori_pitch):.1f}°, "
                f"{math.degrees(self._ori_yaw):.1f}°]\n"
                f"Joints: [{', '.join(f'{a:.2f}' for a in self._current_angles)}]"
            )
            self._mk_pub.publish(self._mk_info)

    # ─── Entry point ────────────────────────────────────────────────────────

    def run(self):
        """Save terminal settings, run the node, restore terminal on exit."""
        import termios
        old_term = termios.tcgetattr(sys.stdin)
        try:
            self._run_inner()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_term)
            self._shutdown_recording()

    def _run_inner(self):
        """Main startup + control loop."""
        import select, termios, tty as _tty
        _tty.setcbreak(sys.stdin.fileno())

        cur_dict   = self._limb.joint_angles()
        cur_angles = [cur_dict.get(n, self.HOME_CONFIG[i])
                      for i, n in enumerate(JOINT_NAMES)]

        print("\n" + "="*62)
        print("  Sawyer Gamepad Teleop — startup")
        print("="*62)
        print(f"  Current joints: {[f'{a:.3f}' for a in cur_angles]}")
        print(f"  Home    joints: {[f'{a:.3f}' for a in self.HOME_CONFIG]}")
        print()
        print("  Press on the controller:")
        print("    [A]  →  move to HOME first  (safe, slow)")
        print("    [B]  →  skip home, start from current position")
        print("="*62 + "\n")

        pygame.event.clear()
        go_home = None
        while go_home is None and not rospy.is_shutdown():
            pygame.event.pump()
            if self._joy.get_button(BTN_A):
                go_home = True
                print("  [A] pressed — moving to HOME safely...\n")
            elif self._joy.get_button(BTN_B):
                go_home = False
                print("  [B] pressed — starting from current position\n")
            rospy.sleep(0.05)

        if go_home:
            self._move_to_home()
            actual = self._limb.joint_angles()
            actual_list = [actual.get(n, self.HOME_CONFIG[i])
                           for i, n in enumerate(JOINT_NAMES)]
            self._current_angles   = actual_list
            self._cmd_angles       = actual_list
            self._last_sent_angles = actual_list
            self.rik.reset(actual_list)
            warmup_angles = actual_list
        else:
            rospy.loginfo("[ctrl] Skipping home — seeding IK from current angles")
            self.rik.reset(cur_angles)
            self._current_angles   = cur_angles
            self._cmd_angles       = cur_angles
            self._last_sent_angles = cur_angles
            warmup_angles = cur_angles

        # ── TF warmup ────────────────────────────────────────────────────
        rospy.loginfo("[ctrl] Warming up TF...")
        r = rospy.Rate(50)
        for _ in range(100):
            self._publish_joint_states(warmup_angles, command_robot=False)
            r.sleep()

        pos, quat = self._lookup_ee()
        if pos is None:
            rospy.logfatal("[ctrl] Cannot read EE TF — is the robot connected?")
            return

        self._home_pos  = pos
        self._home_quat = quat
        rospy.loginfo("[ctrl] EE start: [%.4f, %.4f, %.4f]", *pos)

        self._current_goal = list(pos)
        self._reset_orientation_to_home()
        self._cmd_angles = list(warmup_angles)

        # ── Wait for all buttons released before teleop ───────────────────
        rospy.loginfo("[ctrl] Release all controller buttons to begin teleop...")
        while not rospy.is_shutdown():
            pygame.event.pump()
            if not any(self._joy.get_button(i)
                       for i in range(self._joy.get_numbuttons())):
                break
            rospy.sleep(0.05)
        self._prev_btn = {}
        rospy.loginfo("[ctrl] All buttons released — teleop active")

        print("\n" + "="*62)
        print("  CONTROLS")
        print("="*62)
        print("  Hold [L1]         → enable motion, default mode")
        print("                      Left stick   → X / Y position")
        print("                      Right stick Y → Z (up/down)")
        print("                      Right stick X → YAW")
        print("  Hold [L2]         → enable motion, roll+pitch mode")
        print("                      Left stick   → X / Y position")
        print("                      Right stick X → ROLL")
        print("                      Right stick Y → PITCH  (Z frozen)")
        print("  [X]               → reset orientation to home")
        print("  [Y]               → toggle gripper open/close")
        print("  [Start]           → return to HOME (safe)")
        print("  [Select]          → re-anchor IK at current pos")
        print("  keyboard 'h'      → return to HOME (same as Start)")
        print("  keyboard r/f/d    → record / finish+save / discard episode")
        print("="*62 + "\n")

        dt   = min(1.0 / self.control_rate, 0.05)   # cap dt to avoid huge jumps after lag
        rate = rospy.Rate(self.control_rate)

        # ══════════════════════════════════════════════════════════════════
        # Main control loop
        # ══════════════════════════════════════════════════════════════════
        while not rospy.is_shutdown():
            pygame.event.pump()

            # ── Keyboard: h=HOME  r=record  f=finish+save  d=discard ──────
            if select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1).lower()
                if key == 'h':
                    rospy.loginfo("[ctrl] 'h' — returning to HOME")
                    self._move_to_home()
                    actual = self._limb.joint_angles()
                    al = [actual.get(n, self.HOME_CONFIG[i]) for i, n in enumerate(JOINT_NAMES)]
                    self._current_goal     = list(self._home_pos)
                    self._enabled          = False
                    self._current_angles   = al
                    self._cmd_angles       = al
                    self._last_sent_angles = al
                    self.rik.reset(al)
                elif key in ('r', 'f', 'd'):
                    self._handle_record_key(key)

            # ── [Start] → HOME ────────────────────────────────────────────
            if self._btn_pressed(BTN_START):
                rospy.loginfo("[ctrl] START — returning to HOME")
                self._move_to_home()
                actual = self._limb.joint_angles()
                al = [actual.get(n, self.HOME_CONFIG[i]) for i, n in enumerate(JOINT_NAMES)]
                self._current_goal     = list(self._home_pos)
                self._enabled          = False
                self._current_angles   = al
                self._cmd_angles       = al
                self._last_sent_angles = al
                self.rik.reset(al)

            # ── [Select] → re-anchor IK ───────────────────────────────────
            if self._btn_pressed(BTN_SELECT):
                self.rik.reset(list(self._current_angles))
                cur_pos, _ = self._lookup_ee()
                if cur_pos:
                    self._current_goal = cur_pos
                rospy.loginfo("[ctrl] SELECT — re-anchored at %.3f %.3f %.3f",
                              *self._current_goal)

            # ── [X] → reset orientation ───────────────────────────────────
            if self._btn_pressed(BTN_X):
                self._reset_orientation_to_home()
                rospy.loginfo("[ctrl] Orientation reset to home")

            # ── [Y] → toggle gripper ──────────────────────────────────────
            if self._btn_pressed(BTN_Y) and self._gripper_ready:
                if self._gripper_open:
                    self._gripper.close()
                    self._gripper_open = False
                    rospy.loginfo("[ctrl] Gripper CLOSED")
                else:
                    self._gripper.open()
                    self._gripper_open = True
                    rospy.loginfo("[ctrl] Gripper OPEN")

            # ── L1 / L2 held → enable motion ──────────────────────────────
            l1 = self._btn_held(BTN_L1)
            l2 = self._btn_held(BTN_L2)
            self._enabled  = l1 or l2
            self._ori_mode = 1 if l2 else 0
            if self._enabled:
                rospy.loginfo_throttle(1.0, "[ctrl] ENABLED — %s",
                                       "L2 (roll+pitch)" if l2 else "L1 (yaw+Z)")

            goal_pos = None

            if self._enabled:
                # ── Read sticks ───────────────────────────────────────────
                lx = -self._axis(AXIS_LX)
                ly = -self._axis(AXIS_LY)
                rx =  self._axis(AXIS_RX)
                ry = -self._axis(AXIS_RY)

                has_stick = abs(lx) + abs(ly) + abs(rx) + abs(ry) > 0.0

                if not has_stick:
                    # Sticks centred — read actual hardware angles and hold
                    actual = self._limb.joint_angles()
                    self._current_angles = [
                        actual.get(n, self._current_angles[i])
                        for i, n in enumerate(JOINT_NAMES)]
                    self._publish_joint_states(self._current_angles, command_robot=False)

                else:
                    # ── Integrate EE goal from sticks ─────────────────────
                    new_gx = self._current_goal[0] + ly * XY_VEL_SCALE * dt
                    new_gy = self._current_goal[1] + lx * XY_VEL_SCALE * dt

                    if self._ori_mode == 0:          # L1: Z + yaw
                        new_gz = self._current_goal[2] + ry * Z_VEL_SCALE * dt
                        self._ori_yaw += rx * YAW_VEL_SCALE * dt
                    else:                             # L2: roll + pitch
                        new_gz = self._current_goal[2]
                        self._ori_roll  -= ry * ROLL_VEL_SCALE  * dt
                        self._ori_pitch += rx * PITCH_VEL_SCALE * dt

                    # ── Proximity scale (only Cartesian helper kept) ───────
                    new_gx, new_gy = self._apply_proximity_scale(new_gx, new_gy)

                    gx, gy, gz = new_gx, new_gy, new_gz   # goal moves freely at stick rate

                    # ── Workspace wall clamping ───────────────────────────
                    hx, hy, _ = self._home_pos
                    bowl_floor = float(self._sensor_bowl_pos[2]) + TOOL_LENGTH

                    if gx < hx - 0.40: gx = hx - 0.40; rospy.logwarn_throttle(1.0, "[box] X min")
                    if gx > hx + 0.40: gx = hx + 0.40; rospy.logwarn_throttle(1.0, "[box] X max")
                    if gy < hy - 0.40: gy = hy - 0.40; rospy.logwarn_throttle(1.0, "[box] Y min")
                    if gy > hy + 0.40: gy = hy + 0.40; rospy.logwarn_throttle(1.0, "[box] Y max")
                    if gz < bowl_floor: gz = bowl_floor; rospy.logwarn_throttle(0.5,  "[box] Z floor")

                    self._current_goal = [gx, gy, gz]
                    goal_pos = self._current_goal

                    goal_quat = list(tft.quaternion_from_euler(
                        self._ori_roll, self._ori_pitch, self._ori_yaw))

                    # ── IK → rate-limit → EMA (single joint-space filter) ─
                    try:
                        angles = self.rik.solve_position(
                            positions=goal_pos,
                            orientations=goal_quat,
                            tolerances=[0.0] * 6)

                        if len(angles) == 7 and all(math.isfinite(a) for a in angles):
                            # Warn if IK jumped wildly (helps diagnose singularities)
                            max_delta = max(abs(a - c)
                                           for a, c in zip(angles, self._current_angles))
                            if max_delta > 0.20:
                                rospy.logwarn_throttle(1.0,
                                    "[IK] large jump %.3f rad — possible singularity", max_delta)

                            # Rate-limit each joint
                            clamped = [
                                prev + max(-MAX_JOINT_DELTA,
                                           min(MAX_JOINT_DELTA, new - prev))
                                for prev, new in zip(self._current_angles, angles)
                            ]
                            # Single EMA smoother on joint commands
                            self._cmd_angles = [
                                JOINT_SMOOTH_ALPHA * a + (1.0 - JOINT_SMOOTH_ALPHA) * c
                                for a, c in zip(clamped, self._cmd_angles)
                            ]
                            # Only send to robot if change exceeds deadband
                            if max(abs(a - b) for a, b in
                                   zip(self._cmd_angles, self._last_sent_angles)) > JOINT_CMD_DEADBAND:
                                self._publish_joint_states(self._cmd_angles)
                                self._last_sent_angles = list(self._cmd_angles)

                    except Exception as e:
                        rospy.logwarn_throttle(2.0, "[ctrl] IK failed: %s", e)

            else:
                # Motion disabled — don't keep sending commands, let robot hold
                self._publish_joint_states(self._current_angles, command_robot=False)

            # ── RViz update at 10 Hz (every 5th cycle) ────────────────────
            self._viz_tick = (self._viz_tick + 1) % 5
            if self._viz_tick == 0:
                actual_pos, _ = self._lookup_ee()
                self._publish_markers(goal_pos or self._current_goal, actual_pos, self._enabled)
                self._publish_box()
                self._publish_sensor_bowl()
                self._publish_table()

            rate.sleep()


# ─── ROS node entry point ──────────────────────────────────────────────────

def main():
    node = PS2TeleopVizNode()
    node.run()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass