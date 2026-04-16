#!/usr/bin/env python3
"""
teleop_legion_key.py

Keyboard teleop for swarm robotics applications 

Core behaviors:
- Robots are identified by "robot name" (default = Linux username on robot)
- A robot is considered "available" if there is at least one subscriber on:
    /<robot_name>/cmd_vel
  (i.e., motor_driver_node is running on that robot)

Startup:
- Prints list of available robots, prompts user to pick one.
- Keeps prompting until a valid robot name is entered.

Switching:
- Press 'm' any time to switch robot (same validation loop).

Offline detection:
- While driving, if the current robot stops subscribing to /<robot>/cmd_vel,
  teleop stops and returns to selection prompt.

Tracked robot turning:
- '4' rotates LEFT  (left track backward, right track forward)  -> angular.z positive
- '6' rotates RIGHT (left track forward, right track backward)  -> angular.z negative

Circle turns (smallest radius; ONE TRACK ONLY):
- 7 forward-left  : RIGHT track forward, LEFT track stopped
- 9 forward-right : LEFT track forward, RIGHT track stopped
- 1 backward-left : RIGHT track backward, LEFT track stopped
- 3 backward-right: LEFT track backward, RIGHT track stopped

Terminal reliability note:
- This program reads keys in RAW mode for instant keypress capture.
- Printing while RAW is active can cause newline behavior that "drifts" output rightward
  (because \n may not return to column 0).
- We fix that by ALWAYS printing in cooked mode, then returning to raw.
"""

import re
import sys
import select
import termios
import tty
import time
import json
import os
from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from .audit_logger import AuditLogger
from .drive_profiles import load_profile_registry, resolve_robot_profile


CMD_VEL_RE = re.compile(r"^/([^/]+)/cmd_vel$")
HEARTBEAT_RE = re.compile(r"^/([^/]+)/heartbeat$")


def restore_terminal_settings(settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def set_raw_mode():
    tty.setraw(sys.stdin.fileno())


def read_key_raw(timeout_s: float = 0.1) -> str:
    """
    Read a single keypress (with arrow-key escape sequence support).
    Assumes terminal is already in raw mode.
    """
    rlist, _, _ = select.select([sys.stdin], [], [], timeout_s)
    if not rlist:
        return ""
    key = sys.stdin.read(1)
    if key == "\x1b":  # start of escape sequence
        # Typical arrow keys arrive as ESC [ A/B/C/D (3 chars total including ESC)
        key += sys.stdin.read(2)
    return key


@dataclass
class TeleopState:
    available_robots: List[str]
    active_robot: Optional[str]
    cmd_vel_topic: Optional[str]
    linear_speed: float
    angular_speed: float
    allow_cmd_vel_switching: bool
    wheel_separation: float


class RobotLegionTeleop(Node):
    def __init__(self):
        super().__init__("robot_legion_teleop_python")

        # Initialize audit logger
        self.audit = AuditLogger(self, "teleop", log_file_path="/tmp/teleop_audit.jsonl")

        # Optional parameters
        self.declare_parameter("allow_cmd_vel_switching", True)
        self.allow_cmd_vel_switching = bool(self.get_parameter("allow_cmd_vel_switching").value)
        self.declare_parameter("teleop_debug", True)
        self.teleop_debug = bool(self.get_parameter("teleop_debug").value)

        # Observed robot profiles discovered via heartbeat topics.
        # Mapping: robot_name -> {"drive_type": str, "hardware": str, "t": timestamp}
        self._observed_profiles = {}

        # Heartbeat subscriptions we create dynamically: robot_name -> subscription
        self._hb_subs = {}

        # Timer to refresh heartbeat discovery and to expire stale entries
        self.create_timer(1.0, self._refresh_heartbeats)

        # Profile registry (optional)
        self.declare_parameter("profiles_path", "")
        profiles_path = str(self.get_parameter("profiles_path").value).strip() or None
        self._profile_registry = None
        try:
            self._profile_registry = load_profile_registry(profiles_path)
        except Exception as ex:
            self.get_logger().warning(f"[teleop] Failed to load profile registry: {ex}")

        # Defaults from YAML (ROS params still override)
        drive_defaults = self._get_default_drive_params()
        wheel_sep_default = (
            drive_defaults.get("wheel_separation_m")
            or drive_defaults.get("wheel_base_m")
            or 0.18
        )
        teleop_lin_default = drive_defaults.get("teleop_linear_mps") or 0.5
        teleop_ang_default = drive_defaults.get("teleop_angular_rps") or 1.0
        teleop_step_default = drive_defaults.get("teleop_speed_step") or 1.1
        teleop_medium_steps = int(drive_defaults.get("teleop_medium_steps") or 10)
        teleop_fast_linear_steps = int(drive_defaults.get("teleop_fast_linear_steps") or 15)
        teleop_fast_angular_steps = int(drive_defaults.get("teleop_fast_angular_steps") or 10)
        teleop_omni_turn_gain = float(drive_defaults.get("teleop_omni_turn_gain") or 0.5)
        teleop_smooth_default = float(drive_defaults.get("teleop_smoothing_alpha") or 0.0)

        # Used for circle-turn math (must match motor_driver_node)
        self.declare_parameter("wheel_separation", float(wheel_sep_default))
        self.wheel_separation = float(self.get_parameter("wheel_separation").value)

        # Teleop tuning parameters (YAML default, ROS overrides)
        self.declare_parameter("teleop_linear_mps", float(teleop_lin_default))
        self.declare_parameter("teleop_angular_rps", float(teleop_ang_default))
        self.declare_parameter("teleop_speed_step", float(teleop_step_default))
        self.declare_parameter("teleop_medium_steps", int(teleop_medium_steps))
        self.declare_parameter("teleop_fast_linear_steps", int(teleop_fast_linear_steps))
        self.declare_parameter("teleop_fast_angular_steps", int(teleop_fast_angular_steps))
        self.declare_parameter("teleop_omni_turn_gain", float(teleop_omni_turn_gain))
        self.declare_parameter("teleop_smoothing_alpha", float(teleop_smooth_default))

        # Track whether ROS params were explicitly overridden (best-effort)
        self._param_overrides = {
            "wheel_separation": self.wheel_separation != float(wheel_sep_default),
            "teleop_linear_mps": float(self.get_parameter("teleop_linear_mps").value) != float(teleop_lin_default),
            "teleop_angular_rps": float(self.get_parameter("teleop_angular_rps").value) != float(teleop_ang_default),
            "teleop_speed_step": float(self.get_parameter("teleop_speed_step").value) != float(teleop_step_default),
        }

        # Active robot publishers (used by fpv_camera_mux)
        self.active_robot_pub = self.create_publisher(String, "/active_robot", 10)
        self.active_robot_teleop_pub = self.create_publisher(String, "/teleop/active_robot", 10)
        self._human_override_pubs: Dict[str, object] = {}

        # Publisher created after robot selection
        self.publisher_ = None
        self.cmd_vel_topic: Optional[str] = None
        self.current_robot_name: Optional[str] = None

        # Speed profile (defaults from YAML; can be overridden per-robot)
        self.teleop_omni_turn_gain = float(self.get_parameter("teleop_omni_turn_gain").value)
        # Smoothing for Twist commands (0=off, 1=very smooth)
        self.teleop_smoothing_alpha = float(self.get_parameter("teleop_smoothing_alpha").value)
        self._filtered_twist = Twist()
        self._set_speed_profile(
            linear=float(self.get_parameter("teleop_linear_mps").value),
            angular=float(self.get_parameter("teleop_angular_rps").value),
            step=float(self.get_parameter("teleop_speed_step").value),
            medium_steps=int(self.get_parameter("teleop_medium_steps").value),
            fast_linear_steps=int(self.get_parameter("teleop_fast_linear_steps").value),
            fast_angular_steps=int(self.get_parameter("teleop_fast_angular_steps").value),
        )

        # Republish state
        self.last_lin_mult = 0.0
        self.last_ang_mult = 0.0
        self.last_twist = Twist()
        self.is_moving = False
        self.strafe_mode = False

        # KEYMAP (7/9/1/3 handled specially for one-track-only circles)
        self.move_bindings = {
            # Arrow keys
            "\x1b[A": ("twist", 1, 0),     # up
            "\x1b[B": ("twist", -1, 0),    # down
            "\x1b[D": ("twist", 0, 1),     # left  (rotate left)
            "\x1b[C": ("twist", 0, -1),    # right (rotate right)

            # Numpad
            "8": ("twist", 1, 0),          # forward
            "2": ("twist", -1, 0),         # backward

            # Rotate in place
            "4": ("twist", 0, 1),          # rotate LEFT
            "6": ("twist", 0, -1),         # rotate RIGHT

            # One-track-only circle turns (handled specially)
            "7": ("circle", None, None),   # forward-left  (right track only)
            "9": ("circle", None, None),   # forward-right (left track only)
            "1": ("circle", None, None),   # backward-left (right track only)
            "3": ("circle", None, None),   # backward-right(left track only)
        }

        self.speed_bindings = {
            "w": self._increase_both_speeds,
            "+": self._increase_both_speeds,
            "e": self._decrease_both_speeds,
            "-": self._decrease_both_speeds,
            "q": self._increase_linear_speed,
            "/": self._increase_linear_speed,
            "r": self._decrease_linear_speed,
            "*": self._decrease_linear_speed,
            "i": self._set_slow_profile,
            "o": self._set_medium_profile,
            "p": self._set_fast_profile,
        }

        # Timers
        self.create_timer(0.1, self._republish_last_twist)
        self.create_timer(0.5, self._offline_watchdog)
        # Human-override heartbeat so autonomous executors yield to teleop.
        self.create_timer(0.2, self._publish_human_override)

        # Initial UI
        self._tprint(self.render_instructions(topic_name=None))
        self._tprint("[STARTUP] Discovering robots...")

    # ---------------- Terminal output (cooked mode only) ----------------

    def _tprint(self, text: str = ""):
        """
        Print reliably even while the program uses RAW mode for keypress reading.
        We temporarily restore "cooked" terminal settings for printing.
        """
        # If stdin isn't a tty, just print normally (useful for piping/logging)
        if not sys.stdin.isatty():
            print(text)
            sys.stdout.flush()
            return

        try:
            # Put terminal back to cooked for correct newline behavior
            restore_terminal_settings(self._terminal_settings)
        except Exception:
            # If settings aren't initialized yet, best effort print
            print(text)
            sys.stdout.flush()
            return

        # Normal printing in cooked mode
        if text:
            print(text)
        else:
            print("")
        sys.stdout.flush()

        # Back to raw so key reading continues to work
        try:
            set_raw_mode()
        except Exception:
            pass

    def _dprint(self, text: str):
        if self.teleop_debug:
            self._tprint(f"[DEBUG] {text}")

    # ---------------- ROS graph warm-up (startup discovery fix) ----------------

    def _warmup_discovery(self, timeout_sec: float = 1.0, spin_step_sec: float = 0.1):
        """
        Warm up DDS/ROS graph discovery so get_topic_names_and_types() /
        get_subscriptions_info_by_topic() are correct on first display.

        Why this exists:
        - On startup, the ROS graph cache may not yet include remote peers.
        - If we query immediately, we may see (none found) even though robots exist.
        - A short spin_once loop processes discovery traffic and populates the graph.
        """
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=float(spin_step_sec))
            # Stop early as soon as we see any available robot
            if self.list_available_robots():
                return

    # ---------------- Heartbeat discovery ----------------

    def _heartbeat_callback(self, msg: String, topic_name: str):
        """Callback for per-robot heartbeat subscription. We record the reported
        drive_type and hardware so teleop and other components can adapt
        behavior dynamically without hard-coded robot names in config.
        """
        try:
            data = json.loads(msg.data or "{}")
        except Exception:
            return
        robot = data.get("robot")
        if not robot:
            # Try to extract robot from topic as a fallback
            m = HEARTBEAT_RE.match(topic_name)
            robot = m.group(1) if m else None
        if not robot:
            return
        self._observed_profiles[robot] = {
            "drive_type": data.get("drive_type"),
            "hardware": data.get("hardware"),
            "t": data.get("t", time.time()),
        }

    def _refresh_heartbeats(self):
        """Find heartbeat topics and subscribe to new ones; expire old observed entries."""
        # Discover heartbeat topics in the graph
        topics_and_types = self.get_topic_names_and_types()
        for (t, _types) in topics_and_types:
            m = HEARTBEAT_RE.match(t)
            if not m:
                continue
            robot = m.group(1)
            if robot in self._hb_subs:
                continue
            # Create a subscription capturing the topic name in a lambda
            try:
                sub = self.create_subscription(String, t, lambda msg, tn=t: self._heartbeat_callback(msg, tn), 10)
                self._hb_subs[robot] = sub
            except Exception:
                pass

        # Expire observed entries older than a few seconds
        now = time.time()
        expired = [r for r, v in self._observed_profiles.items() if now - float(v.get("t", now)) > 5.0]
        for r in expired:
            self._observed_profiles.pop(r, None)

    # ---------------- State for easy web integration ----------------

    def get_state(self) -> TeleopState:
        robots = self.list_available_robots()
        return TeleopState(
            available_robots=robots,
            active_robot=self.current_robot_name,
            cmd_vel_topic=self.cmd_vel_topic,
            linear_speed=self.linear_speed,
            angular_speed=self.angular_speed,
            allow_cmd_vel_switching=self.allow_cmd_vel_switching,
            wheel_separation=self.wheel_separation,
        )

    def get_state_dict(self) -> Dict:
        """
        JSON-friendly dict version of state (plug-and-play for future HTTP/WebSocket API).
        """
        s = self.get_state()
        return {
            "available_robots": s.available_robots,
            "active_robot": s.active_robot,
            "cmd_vel_topic": s.cmd_vel_topic,
            "linear_speed": s.linear_speed,
            "angular_speed": s.angular_speed,
            "allow_cmd_vel_switching": s.allow_cmd_vel_switching,
            "wheel_separation": s.wheel_separation,
        }

    # ---------------- Renderers (return strings; terminal/web can reuse) ----------------

    def render_instructions(self, topic_name: Optional[str]) -> str:
        lines = []
        lines.append("-------------------------------")
        lines.append("      ROBOT LEGION TELEOP      ")
        lines.append("-------------------------------")
        if topic_name:
            lines.append(f"Publishing Twist on: {topic_name}")
        else:
            lines.append("SELECT A ROBOT")
        lines.append(f"allow_cmd_vel_switching: {self.allow_cmd_vel_switching}")
        lines.append(f"wheel_separation (for circle turns): {self.wheel_separation:.2f} m")
        if self.current_robot_name:
            lines.append("")
            lines.append(f"ACTIVE ROBOT: {self.current_robot_name}")
            lines.append(f"DRIVE MODE: {'STRAFE' if self.strafe_mode else 'NORMAL'}")
        lines.append("")
        lines.append("MOVEMENT:")
        if self.strafe_mode:
            lines.append("  8 forward")
            lines.append("  2 backward")
            lines.append("  4 strafe-left")
            lines.append("  6 strafe-right")
            lines.append("  7 strafe forward-left")
            lines.append("  9 strafe forward-right")
            lines.append("  1 strafe backward-left")
            lines.append("  3 strafe backward-right")
        else:
            lines.append("  8 forward")
            lines.append("  2 backward")
            lines.append("  4 rotate-left")
            lines.append("  6 rotate-right")
            lines.append("  7 circle forward-left")
            lines.append("  9 circle forward-right")
            lines.append("  1 circle backward-left")
            lines.append("  3 circle backward-right")
        lines.append("")
        lines.append("ARROW KEYS ALSO WORK")
        lines.append(" [UP] forward")
        lines.append(" [DOWN] backward")
        lines.append(" [LEFT] rotate left")
        lines.append(" [RIGHT] rotate right")
        lines.append("")
        lines.append("STOP:")
        lines.append("  [SPACE] OR 's' OR 5")
        lines.append("")
        lines.append("SPEED PROFILES:")
        lines.append("  i slow, o medium, p fast")
        lines.append("")
        lines.append("Speed scaling:")
        lines.append("  q OR / increase linear")
        lines.append("  r OR * decrease linear")
        lines.append("  w OR + increase speed")
        lines.append("  e OR - decrease speed")
        lines.append("")
        lines.append("Robot selection:")
        lines.append("  m choose robot")
        lines.append("  0 toggle strafe mode (mecanum/omni only)")
        lines.append("")
        lines.append("CTRL-C to quit.")
        lines.append("-------------------------------")
        lines.append(self.render_current_speeds())
        return "\n".join(lines)

    def render_current_speeds(self) -> str:
        return f"Linear Speed: {self.linear_speed:.2f}  Angular Speed: {self.angular_speed:.2f}"

    def render_robot_list(self, robots: List[str]) -> str:
        lines = []
        lines.append("")
        lines.append("======= AVAILABLE ROBOTS =======")
        if not robots:
            lines.append("")
            lines.append("  (none found)")
            lines.append("")
            lines.append("  - Try 'r' to refresh")
            lines.append("")
            lines.append("  - Make sure each robot is")
            lines.append("    on the same ROS_DOMAIN_ID")
            lines.append("    and network.")
            lines.append("")
            lines.append("  - Make sure your robots are")
            lines.append("    currently running at least")
            lines.append("    one node.")
            lines.append("")
        else:
            lines.append("")
            for r in robots:
                lines.append(f"  - {r}")
                lines.append("")
        lines.append("================================")
        lines.append("")
        return "\n".join(lines)

    # ---------------- Discovery helpers ----------------

    def _topic_to_robot(self, topic: str) -> Optional[str]:
        m = CMD_VEL_RE.match(topic)
        return m.group(1) if m else None

    def _discover_candidate_cmd_vel_topics(self) -> List[str]:
        topics_and_types = self.get_topic_names_and_types()
        topics = [t for (t, _types) in topics_and_types]
        return [t for t in topics if CMD_VEL_RE.match(t)]

    def list_available_robots(self) -> List[str]:
        robots: Set[str] = set()
        for t in self._discover_candidate_cmd_vel_topics():
            subs = self.get_subscriptions_info_by_topic(t)
            if subs and len(subs) > 0:
                robot = self._topic_to_robot(t)
                if robot:
                    robots.add(robot)
        return sorted(robots)

    def _validate_robot_name(self, name: str) -> Tuple[bool, str]:
        name = name.strip()
        if not name:
            return False, ""
        topic = f"/{name}/cmd_vel"
        subs = self.get_subscriptions_info_by_topic(topic)
        if subs and len(subs) > 0:
            return True, topic
        return False, topic

    # ---------------- Active robot helpers ----------------

    def _publish_active_robot(self):
        if not self.current_robot_name:
            return
        msg = String()
        msg.data = self.current_robot_name
        self.active_robot_pub.publish(msg)
        self.active_robot_teleop_pub.publish(msg)

    def _get_default_drive_params(self) -> Dict[str, float]:
        """Return drive params for the default drive profile, if available."""
        if not self._profile_registry:
            return {}
        defaults = self._profile_registry.get("defaults", {}) or {}
        drive_name = defaults.get("drive_profile")
        drive_profiles = self._profile_registry.get("drive_profiles", {}) or {}
        drive = drive_profiles.get(drive_name, {}) if drive_name else {}
        return drive.get("params", {}) or {}

    def _get_robot_drive_params(self, robot_name: str) -> Dict[str, float]:
        if not self._profile_registry:
            return {}
        try:
            prof = resolve_robot_profile(self._profile_registry, robot_name)
            return prof.get("drive_params", {}) or {}
        except Exception:
            return {}

    def _set_speed_profile(
        self,
        linear: float,
        angular: float,
        step: float,
        medium_steps: int,
        fast_linear_steps: int,
        fast_angular_steps: int,
    ):
        self.linear_speed = float(linear)
        self.angular_speed = float(angular)
        self.speed_step = float(step)

        self.base_slow_linear = self.linear_speed
        self.base_slow_angular = self.angular_speed

        self.medium_linear = self.base_slow_linear * (self.speed_step ** int(medium_steps))
        self.medium_angular = self.base_slow_angular * (self.speed_step ** int(medium_steps))
        self.fast_linear = self.base_slow_linear * (self.speed_step ** int(fast_linear_steps))
        self.fast_angular = self.base_slow_angular * (self.speed_step ** int(fast_angular_steps))

    @staticmethod
    def _is_mecanum_drive_type(drive_type: str) -> bool:
        dt = str(drive_type or "").strip().lower()
        return dt in ("mecanum", "omni", "omnidirectional", "mecanum_drive", "mecanum-drive")

    def _get_robot_drive_type(self, robot_name: Optional[str]) -> str:
        if not robot_name:
            return ""

        observed = self._observed_profiles.get(robot_name) or {}
        observed_dt = str(observed.get("drive_type") or "").strip().lower()
        if observed_dt:
            return observed_dt

        if self._profile_registry:
            try:
                prof = resolve_robot_profile(self._profile_registry, robot_name)
                return str(prof.get("drive_type") or "").strip().lower()
            except Exception:
                pass
        return ""

    def _current_robot_supports_strafe(self) -> bool:
        return self._is_mecanum_drive_type(self._get_robot_drive_type(self.current_robot_name))

    def _strafe_diagnostics(self, robot_name: Optional[str]) -> Dict[str, str]:
        if not robot_name:
            return {
                "robot": "",
                "observed_drive_type": "",
                "registry_drive_type": "",
                "effective_drive_type": "",
                "supports_strafe": "False",
                "source": "none",
            }

        observed = self._observed_profiles.get(robot_name) or {}
        observed_dt = str(observed.get("drive_type") or "").strip().lower()
        registry_dt = ""
        if self._profile_registry:
            try:
                prof = resolve_robot_profile(self._profile_registry, robot_name)
                registry_dt = str(prof.get("drive_type") or "").strip().lower()
            except Exception:
                registry_dt = ""

        effective_dt = observed_dt or registry_dt
        source = "heartbeat" if observed_dt else ("registry" if registry_dt else "unknown")
        supports = self._is_mecanum_drive_type(effective_dt)
        return {
            "robot": robot_name,
            "observed_drive_type": observed_dt,
            "registry_drive_type": registry_dt,
            "effective_drive_type": effective_dt,
            "supports_strafe": str(bool(supports)),
            "source": source,
        }

    def _toggle_strafe_mode(self):
        if self.publisher_ is None or not self.current_robot_name:
            self._tprint("[STRAFE] No robot selected.")
            return

        diag = self._strafe_diagnostics(self.current_robot_name)
        self._dprint(
            "strafe-toggle request "
            f"robot={diag['robot']} "
            f"observed_drive_type={diag['observed_drive_type'] or '<none>'} "
            f"registry_drive_type={diag['registry_drive_type'] or '<none>'} "
            f"effective_drive_type={diag['effective_drive_type'] or '<none>'} "
            f"source={diag['source']} "
            f"supports_strafe={diag['supports_strafe']}"
        )

        if diag["supports_strafe"] != "True":
            self.strafe_mode = False
            self._tprint(
                f"[STRAFE] Disabled: robot '{self.current_robot_name}' is not mecanum/omni "
                f"(effective drive_type='{diag['effective_drive_type'] or 'unknown'}')."
            )
            self._tprint(
                "[STRAFE] Check robot_profiles.yaml drive_profile.type and/or heartbeat_node drive_type."
            )
            return

        self.strafe_mode = not self.strafe_mode

        # Stop immediately while changing modes to avoid unexpected motion.
        self._publish_and_log_twist(Twist(), "strafe-mode-toggle-stop")
        self.is_moving = False
        self.last_lin_mult = 0.0
        self.last_ang_mult = 0.0
        self.last_twist = Twist()
        self._tprint(f"[STRAFE] {'ENABLED' if self.strafe_mode else 'DISABLED'}")

    def _apply_robot_profile(self, robot_name: str):
        """Update teleop settings based on YAML for this robot (unless ROS overrides are set)."""
        drive_params = self._get_robot_drive_params(robot_name)
        if not drive_params:
            return

        if not self._param_overrides.get("wheel_separation"):
            wheel_sep = (
                drive_params.get("wheel_separation_m")
                or drive_params.get("wheel_base_m")
                or self.wheel_separation
            )
            self.wheel_separation = float(wheel_sep)

        if any(self._param_overrides.get(k) for k in ("teleop_linear_mps", "teleop_angular_rps", "teleop_speed_step")):
            return

        teleop_lin = drive_params.get("teleop_linear_mps") or self.linear_speed
        teleop_ang = drive_params.get("teleop_angular_rps") or self.angular_speed
        teleop_step = drive_params.get("teleop_speed_step") or self.speed_step
        teleop_medium_steps = int(drive_params.get("teleop_medium_steps") or 10)
        teleop_fast_linear_steps = int(drive_params.get("teleop_fast_linear_steps") or 15)
        teleop_fast_angular_steps = int(drive_params.get("teleop_fast_angular_steps") or 10)
        teleop_omni_turn_gain = float(drive_params.get("teleop_omni_turn_gain") or self.teleop_omni_turn_gain)
        teleop_smooth = float(drive_params.get("teleop_smoothing_alpha") or self.teleop_smoothing_alpha)

        self._set_speed_profile(
            linear=teleop_lin,
            angular=teleop_ang,
            step=teleop_step,
            medium_steps=teleop_medium_steps,
            fast_linear_steps=teleop_fast_linear_steps,
            fast_angular_steps=teleop_fast_angular_steps,
        )
        self.teleop_omni_turn_gain = teleop_omni_turn_gain
        self.teleop_smoothing_alpha = teleop_smooth

    # ---------------- Topic application ----------------

    def _apply_cmd_vel_topic(self, new_topic: str):
        if not new_topic.startswith("/"):
            new_topic = "/" + new_topic

        if new_topic == "/cmd_vel":
            self._tprint("[ERROR] Refusing to use /cmd_vel. Use /<robot>/cmd_vel for multi-robot architecture.")
            return

        robot = self._topic_to_robot(new_topic)
        if robot:
            self.current_robot_name = robot

        if self.publisher_ is not None:
            try:
                self.destroy_publisher(self.publisher_)
            except Exception:
                pass

        self.publisher_ = self.create_publisher(Twist, new_topic, 10)
        self.cmd_vel_topic = new_topic

        self._publish_active_robot()
        if self.current_robot_name:
            self._apply_robot_profile(self.current_robot_name)
        self.strafe_mode = False
        diag = self._strafe_diagnostics(self.current_robot_name)
        self._dprint(
            "active-robot profile "
            f"robot={diag['robot']} "
            f"observed_drive_type={diag['observed_drive_type'] or '<none>'} "
            f"registry_drive_type={diag['registry_drive_type'] or '<none>'} "
            f"effective_drive_type={diag['effective_drive_type'] or '<none>'} "
            f"source={diag['source']} "
            f"supports_strafe={diag['supports_strafe']}"
        )
        self._tprint(self.render_instructions(new_topic))
        self._tprint(f"[ACTIVE ROBOT] Now controlling: {self.current_robot_name}")

    def _get_or_make_human_override_pub(self, robot: str):
        topic = f"/{robot}/human_override"
        pub = self._human_override_pubs.get(topic)
        if pub is None:
            pub = self.create_publisher(String, topic, 10)
            self._human_override_pubs[topic] = pub
        return pub

    def _publish_human_override(self):
        if not self.current_robot_name:
            return
        msg = String()
        msg.data = json.dumps(
            {
                "source": "teleop",
                "active": True,
                "ts": time.time(),
            },
            separators=(",", ":"),
        )
        try:
            self._get_or_make_human_override_pub(self.current_robot_name).publish(msg)
        except Exception:
            pass

    # ---------------- Circle-turn helper ----------------

    def _publish_one_track_circle(self, v_left: float, v_right: float):
        """
        Force one-track-only motion by solving for Twist that yields the desired track speeds:

            v_left  = v - w*L/2
            v_right = v + w*L/2

        Inverse:
            v = (v_left + v_right)/2
            w = (v_right - v_left)/L
        """
        L = self.wheel_separation if self.wheel_separation > 1e-6 else 0.18
        v = 0.5 * (v_left + v_right)
        w = (v_right - v_left) / L

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w

        self.last_twist = twist
        self.is_moving = True

        self._publish_and_log_twist(twist, "one-track-circle")

    # ---------------- Republish + Offline watchdog ----------------

    def _republish_last_twist(self):
        if not self.is_moving or self.publisher_ is None:
            return

        if (
            abs(self.last_twist.linear.x) > 1e-9
            or abs(self.last_twist.linear.y) > 1e-9
            or abs(self.last_twist.angular.z) > 1e-9
        ):
            self._publish_and_log_twist(self.last_twist, "republish")
            return

        if self.last_lin_mult == 0.0 and self.last_ang_mult == 0.0:
            return

        twist = Twist()
        twist.linear.x = self.linear_speed * self.last_lin_mult
        twist.angular.z = self.angular_speed * self.last_ang_mult
        self._publish_and_log_twist(twist, "republish")

    def _offline_watchdog(self):
        """
        If current cmd_vel topic has no subscribers, the robot is "offline".
        Stop and force re-selection.
        """
        if self.publisher_ is None or not self.cmd_vel_topic or not self.current_robot_name:
            return

        subs = self.get_subscriptions_info_by_topic(self.cmd_vel_topic)
        if subs and len(subs) > 0:
            return

        # Robot went offline
        self._tprint("")
        self._tprint(f"[OFFLINE] Robot '{self.current_robot_name}' appears offline (no subscribers on {self.cmd_vel_topic}).")
        self._tprint("[OFFLINE] Stopping and returning to robot selection.")
        try:
            self._publish_and_log_twist(Twist(), "offline-stop")
        except Exception:
            pass

        self.is_moving = False
        self.last_lin_mult = 0.0
        self.last_ang_mult = 0.0
        self.last_twist = Twist()

        # Destroy publisher so main loop forces selection again
        try:
            self.destroy_publisher(self.publisher_)
        except Exception:
            pass
        self.publisher_ = None
        self.cmd_vel_topic = None
        self.current_robot_name = None

    def _publish_and_log_twist(self, twist: Twist, description: str = ""):
        """Publish a Twist command and log to audit trail."""
        if self.publisher_ is None or self.current_robot_name is None:
            return

        # Optional smoothing to avoid harsh command spikes that stress hardware.
        # We bypass smoothing for explicit stops so the robot can halt immediately.
        if self.teleop_smoothing_alpha > 0.0:
            is_stop = (
                abs(twist.linear.x) < 1e-9
                and abs(twist.linear.y) < 1e-9
                and abs(twist.angular.z) < 1e-9
            )
            if not is_stop:
                a = max(0.0, min(1.0, float(self.teleop_smoothing_alpha)))
                self._filtered_twist.linear.x = a * twist.linear.x + (1.0 - a) * self._filtered_twist.linear.x
                self._filtered_twist.linear.y = a * twist.linear.y + (1.0 - a) * self._filtered_twist.linear.y
                self._filtered_twist.angular.z = a * twist.angular.z + (1.0 - a) * self._filtered_twist.angular.z
                twist = self._filtered_twist
            else:
                # Reset filter on stop to avoid "coasting" after a full stop.
                self._filtered_twist = Twist()

        try:
            self.publisher_.publish(twist)
            # Log to audit trail
            self.audit.log_command(
                robot=self.current_robot_name,
                source="teleop",
                command_id="twist",
                parameters={
                    "linear_x": float(twist.linear.x),
                    "linear_y": float(twist.linear.y),
                    "angular_z": float(twist.angular.z),
                    "description": description,
                },
                status="sent",
            )
        except Exception as e:
            self.get_logger().error(f"Error publishing Twist: {e}")

    # ---------------- Speed modifiers ----------------

    def _increase_both_speeds(self):
        self.linear_speed *= self.speed_step
        self.angular_speed *= self.speed_step
        self._tprint(self.render_current_speeds())

    def _decrease_both_speeds(self):
        self.linear_speed /= self.speed_step
        self.angular_speed /= self.speed_step
        self._tprint(self.render_current_speeds())

    def _increase_linear_speed(self):
        self.linear_speed *= self.speed_step
        self._tprint(self.render_current_speeds())

    def _decrease_linear_speed(self):
        self.linear_speed /= self.speed_step
        self._tprint(self.render_current_speeds())

    def _set_slow_profile(self):
        self.linear_speed = self.base_slow_linear
        self.angular_speed = self.base_slow_angular
        self._tprint(self.render_current_speeds())

    def _set_medium_profile(self):
        self.linear_speed = self.medium_linear
        self.angular_speed = self.medium_angular
        self._tprint(self.render_current_speeds())

    def _set_fast_profile(self):
        self.linear_speed = self.fast_linear
        self.angular_speed = self.fast_angular
        self._tprint(self.render_current_speeds())

    # ---------------- Robot selection prompt ----------------

    def _prompt_until_valid_robot(self):
        """
        Print robots, prompt until user enters a valid robot name.
        """
        first_pass = True

        while rclpy.ok():
            # Warm up discovery so the initial list is correct on startup.
            if first_pass:
                self._warmup_discovery(timeout_sec=1.0)
                first_pass = False

            robots = self.list_available_robots()
            self._tprint(self.render_robot_list(robots))

            # We MUST be in cooked mode for input()
            restore_terminal_settings(self._terminal_settings)
            try:
                user_input = input("[SELECT ROBOT] Enter robot name \n(or enter 'r' to refresh): ").strip()
            except Exception:
                # Return to raw and retry
                set_raw_mode()
                continue

            # Back to raw for key reading after input()
            set_raw_mode()

            if user_input.lower() == "r":
                # Also warm up on refresh so the very next listing is accurate.
                self._warmup_discovery(timeout_sec=0.5)
                continue

            ok, topic = self._validate_robot_name(user_input)
            if not ok:
                self._tprint(f"[INVALID] '{user_input}' is not an available robot (no subscriber on {topic}). Try again.\n")
                continue

            if not self.allow_cmd_vel_switching and self.publisher_ is not None:
                self._tprint("[ROBOT SWITCH] Switching disabled (allow_cmd_vel_switching:=False).")
                return

            self._apply_cmd_vel_topic(topic)
            return

    # ---------------- Main loop ----------------

    def run(self):
        # Save terminal settings once so all printing can temporarily restore them.
        self._terminal_settings = termios.tcgetattr(sys.stdin)

        # Enter raw mode for key reading
        set_raw_mode()

        # initial selection (uses input, which temporarily flips to cooked itself)
        self._prompt_until_valid_robot()

        try:
            while rclpy.ok():
                key = read_key_raw(timeout_s=0.1)
                if key == "":
                    continue
                if key == "\x03":  # Ctrl-C
                    break

                # If we currently have no robot selected (offline or startup), force selection
                if self.publisher_ is None:
                    if key == "m":
                        self._prompt_until_valid_robot()
                    elif key == "0":
                        self._dprint("key='0' received with no active robot")
                        self._toggle_strafe_mode()
                    elif key in self.speed_bindings:
                        self.speed_bindings[key]()
                    elif key in (" ", "5", "s"):
                        self._tprint("[STOP] (no robot selected)")
                    else:
                        self._tprint("[INFO] No robot selected. Press 'm' to select a robot.")
                    continue

                # Movement keys
                if key in self.move_bindings:
                    mode, lin_mult, ang_mult = self.move_bindings[key]

                    if self.strafe_mode and key in ("8", "2", "4", "6", "7", "9", "1", "3"):
                        if not self._current_robot_supports_strafe():
                            self.strafe_mode = False
                            self._tprint(f"[STRAFE] Disabled: robot '{self.current_robot_name}' is not mecanum/omni.")
                            continue

                        S = self.linear_speed
                        twist = Twist()
                        if key == "8":
                            twist.linear.x = +S
                        elif key == "2":
                            twist.linear.x = -S
                        elif key == "4":
                            twist.linear.y = +S
                        elif key == "6":
                            twist.linear.y = -S
                        elif key == "7":
                            twist.linear.x = +S
                            twist.linear.y = +S
                        elif key == "9":
                            twist.linear.x = +S
                            twist.linear.y = -S
                        elif key == "1":
                            twist.linear.x = -S
                            twist.linear.y = +S
                        elif key == "3":
                            twist.linear.x = -S
                            twist.linear.y = -S

                        self.last_twist = twist
                        self.is_moving = True
                        self._publish_and_log_twist(twist, f"strafe-{key}")
                        continue

                    if mode == "circle":
                        # Circle/one-track semantics differ by drive type. For diff_drive
                        # we synthesize a one-track circle via left/right track speeds.
                        # For omni/mecanum, convert to a safer lateral+rotational motion.
                        S = self.linear_speed
                        drive_type = self._get_robot_drive_type(self.current_robot_name) or "diff_drive"

                        if drive_type in ("diff_drive", "diff", "diff-drive"):
                            if key == "7":
                                self._publish_one_track_circle(v_left=0.0, v_right=+S)
                            elif key == "9":
                                self._publish_one_track_circle(v_left=+S, v_right=0.0)
                            elif key == "1":
                                self._publish_one_track_circle(v_left=0.0, v_right=-S)
                            elif key == "3":
                                self._publish_one_track_circle(v_left=-S, v_right=0.0)
                        else:
                            # Omni/mecanum: emulate circle via a combination of forward/back
                            # and a small angular velocity. Keep behaviors intuitive for user.
                            twist = Twist()
                            if key == "7":
                                twist.linear.x = +S
                                twist.angular.z = +self.teleop_omni_turn_gain * self.angular_speed
                            elif key == "9":
                                twist.linear.x = +S
                                twist.angular.z = -self.teleop_omni_turn_gain * self.angular_speed
                            elif key == "1":
                                twist.linear.x = -S
                                twist.angular.z = +self.teleop_omni_turn_gain * self.angular_speed
                            elif key == "3":
                                twist.linear.x = -S
                                twist.angular.z = -self.teleop_omni_turn_gain * self.angular_speed
                            self.last_twist = twist
                            self.is_moving = True
                            self._publish_and_log_twist(twist, f"circle-{key}")
                        continue

                    # Normal twist
                    self.last_lin_mult = float(lin_mult)
                    self.last_ang_mult = float(ang_mult)

                    twist = Twist()
                    twist.linear.x = self.linear_speed * self.last_lin_mult
                    twist.angular.z = self.angular_speed * self.last_ang_mult

                    self.last_twist = twist
                    self.is_moving = True
                    self._publish_and_log_twist(twist, "normal-twist")
                    continue

                # Stop
                if key in (" ", "5", "s"):
                    self._publish_and_log_twist(Twist(), "stop")
                    self.is_moving = False
                    self.last_lin_mult = 0.0
                    self.last_ang_mult = 0.0
                    self.last_twist = Twist()
                    self._tprint("[STOP]")
                    continue

                # Switch robot
                if key == "m":
                    self._prompt_until_valid_robot()
                    continue

                # Strafe mode toggle
                if key == "0":
                    self._dprint("key='0' received with active robot")
                    self._toggle_strafe_mode()
                    continue

                # Speed controls
                if key in self.speed_bindings:
                    self.speed_bindings[key]()
                    continue

        finally:
            try:
                if self.publisher_ is not None:
                    self._publish_and_log_twist(Twist(), "cleanup")
                if self.current_robot_name:
                    msg = String()
                    msg.data = json.dumps(
                        {
                            "source": "teleop",
                            "active": False,
                            "ts": time.time(),
                        },
                        separators=(",", ":"),
                    )
                    self._get_or_make_human_override_pub(self.current_robot_name).publish(msg)
            except Exception:
                pass
            restore_terminal_settings(self._terminal_settings)
            # Clean up audit logger
            if hasattr(self, 'audit') and self.audit:
                self.audit.close()


def main(args=None):
    rclpy.init(args=args)
    node = RobotLegionTeleop()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
