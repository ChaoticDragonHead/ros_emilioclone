#!/usr/bin/env python3
"""
terminal_orchestrator.py

Terminal menu orchestrator for Robot Legion playbooks.

Goals:
- Keep UI friendly for tall, narrow terminals.
- Provide simple playbook forms with variable prompts.
- Send ExecutePlaybook goals to one robot or all discovered robots.
- Use consistent, rough distance/time math across robots.
"""

from __future__ import annotations

import json
import math
import os
import re
import select
import sys
import termios
import time
import tty
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

try:
    from fleet_orchestrator_interfaces.action import ExecutePlaybook
    HAS_FLEET_ACTION = True
except Exception:
    HAS_FLEET_ACTION = False

    class ExecutePlaybook:  # type: ignore[override]
        class Goal:
            def __init__(self):
                self.intent_id = ""
                self.command_id = ""
                self.vehicle_ids: List[str] = []
                self.parameters_json = ""
                self.north_m = 0.0
                self.east_m = 0.0
from .drive_profiles import load_profile_registry, resolve_robot_profile
from .playbook_strategies import compile_transit_xy_plans


ACTION_RE = re.compile(r"^/([^/]+)/execute_playbook$")
ACTION_SEND_GOAL_SVC_RE = re.compile(r"^/([^/]+)/execute_playbook/_action/send_goal$")
CMD_TOPIC_RE = re.compile(r"^/([^/]+)/execute_playbook_cmd$")
ACTION_TYPE = "fleet_orchestrator_interfaces/action/ExecutePlaybook"
ACTION_SEND_GOAL_TYPE = "fleet_orchestrator_interfaces/action/ExecutePlaybook_SendGoal"
CMD_TOPIC_TYPE = "std_msgs/msg/String"
UI_WIDTH = 31  # Match teleop's narrow terminal style.


@dataclass
class GoalReport:
    robot: str
    accepted: bool
    success: bool
    reason: str


@dataclass
class TwoLegPlan:
    """
    Two-segment robot-relative motion plan:
      rotate(theta1) -> transit(length1) -> rotate(theta2-theta1) -> transit(length2)
    """
    theta1: float
    theta2: float
    length1_m: float
    length2_m: float
    max_detour_m: float


class TerminalOrchestrator(Node):
    def __init__(self):
        super().__init__("terminal_orchestrator")

        # Rough motion constants used for distance -> duration estimates.
        self.declare_parameter("base_linear_mps", 0.40)
        self.declare_parameter("base_strafe_mps", 0.35)
        self.declare_parameter("base_angular_rps", 2.00)
        self.declare_parameter("default_speed_scale", 1.0)
        # Duration calibration scales for meter/degree playbooks.
        # final_duration = ideal_duration * scale
        self.declare_parameter("linear_duration_scale", 1.0)
        self.declare_parameter("angular_duration_scale", 1.0)
        # If dispatch waits longer than this, that robot is skipped for the phase.
        # Requested behavior: explicit 1 second skip threshold.
        self.declare_parameter("dispatch_timeout_s", 1.0)
        self.declare_parameter("send_goal_response_timeout_s", 1.0)
        # Reachability probe used for menu listing so stale graph entries are hidden.
        self.declare_parameter("reachable_probe_timeout_s", 0.15)
        self.declare_parameter("human_override_ttl_s", 0.8)

        self.base_linear_mps = float(self.get_parameter("base_linear_mps").value)
        self.base_strafe_mps = float(self.get_parameter("base_strafe_mps").value)
        self.base_angular_rps = float(self.get_parameter("base_angular_rps").value)
        self.default_speed_scale = float(self.get_parameter("default_speed_scale").value)
        self.linear_duration_scale = float(self.get_parameter("linear_duration_scale").value)
        self.angular_duration_scale = float(self.get_parameter("angular_duration_scale").value)
        self.dispatch_timeout_s = float(self.get_parameter("dispatch_timeout_s").value)
        self.send_goal_response_timeout_s = float(self.get_parameter("send_goal_response_timeout_s").value)
        self.reachable_probe_timeout_s = float(self.get_parameter("reachable_probe_timeout_s").value)
        self.human_override_ttl_s = float(self.get_parameter("human_override_ttl_s").value)

        # Do not use name "_clients": rclpy.Node already uses that internally.
        self._action_clients: Dict[str, ActionClient] = {}
        self._active_goal_handles: Dict[str, object] = {}
        self._cmd_vel_pubs: Dict[str, object] = {}
        self._playbook_cmd_pubs: Dict[str, object] = {}
        self._playbook_result_subs: Dict[str, object] = {}
        self._topic_results: Dict[Tuple[str, str], GoalReport] = {}
        self._human_override_subs: Dict[str, object] = {}
        self._human_override_last_active: Dict[str, float] = {}
        self._last_saved_preset_name: str = ""
        self._in_emergency_stop = False
        self.selected_robot: Optional[str] = None  # None => all robots

        # Fleet-style JSON event emission for integration testing.
        self.declare_parameter("publish_fleet_topics", False)
        self.publish_fleet_topics = bool(self.get_parameter("publish_fleet_topics").value)
        self.task_pub = self.create_publisher(String, "/fo/task", 10)
        self.audit_pub = self.create_publisher(String, "/fo/audit", 10)

        # Best-effort registry for robot type/hardware previews.
        self.declare_parameter("profiles_path", "")
        profiles_path = str(self.get_parameter("profiles_path").value).strip() or None
        self._profile_registry = None
        try:
            self._profile_registry = load_profile_registry(profiles_path)
        except Exception:
            self._profile_registry = None

    # ---------------- UI helpers ----------------

    def _warmup_discovery(self, timeout_sec: float = 1.0, spin_step_sec: float = 0.1):
        """
        Allow ROS graph discovery to populate before first menu render.
        """
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=float(spin_step_sec))
            if self._discover_action_servers():
                return

    def _clear(self):
        # ANSI clear + home cursor.
        sys.stdout.write("\033[2J\033[H")
        sys.stdout.flush()

    def _divider(self) -> str:
        return "-" * UI_WIDTH

    def _fit(self, text: str) -> List[str]:
        text = str(text)
        if len(text) <= UI_WIDTH:
            return [text]
        words = text.split(" ")
        out: List[str] = []
        cur = ""
        for w in words:
            if not cur:
                cur = w
            elif len(cur) + 1 + len(w) <= UI_WIDTH:
                cur += " " + w
            else:
                out.append(cur)
                cur = w
        if cur:
            out.append(cur)
        return out

    def _print_screen(self, title: str, lines: List[str]):
        self._clear()
        print(self._divider())
        for ln in self._fit(title[:UI_WIDTH]):
            print(ln)
        print(self._divider())
        for ln in lines:
            for part in self._fit(ln):
                print(part)
        print(self._divider())

    def _pause(self, text: str = "Press Enter..."):
        input((text + " ").strip())

    def _is_stop_cmd(self, raw: str) -> bool:
        s_raw = str(raw or "")
        s = s_raw.strip().lower()
        return s in ("!", "stop", "kill", "k", "5") or s_raw == " "

    def _get_or_make_cmd_vel_pub(self, robot: str):
        topic = f"/{robot}/cmd_vel"
        pub = self._cmd_vel_pubs.get(topic)
        if pub is None:
            pub = self.create_publisher(Twist, topic, 10)
            self._cmd_vel_pubs[topic] = pub
        return pub

    def _get_or_make_playbook_cmd_pub(self, robot: str):
        topic = f"/{robot}/execute_playbook_cmd"
        pub = self._playbook_cmd_pubs.get(topic)
        if pub is None:
            pub = self.create_publisher(String, topic, 10)
            self._playbook_cmd_pubs[topic] = pub
        return pub

    def _ensure_playbook_result_sub(self, robot: str):
        topic = f"/{robot}/execute_playbook_result"
        if topic in self._playbook_result_subs:
            return self._playbook_result_subs[topic]

        def _cb(msg: String, _robot=robot):
            try:
                payload = json.loads(msg.data or "{}")
            except Exception:
                return
            intent_id = str(payload.get("intent_id", "")).strip()
            if not intent_id:
                return
            success = bool(payload.get("success", False))
            reason = str(payload.get("reason", "")).strip() or ("ok" if success else "failed")
            report = GoalReport(
                robot=_robot,
                accepted=True,
                success=success,
                reason=reason,
            )
            self._topic_results[(_robot, intent_id)] = report

        sub = self.create_subscription(String, topic, _cb, 50)
        self._playbook_result_subs[topic] = sub
        return sub

    def _ensure_human_override_sub(self, robot: str):
        topic = f"/{robot}/human_override"
        if topic in self._human_override_subs:
            return self._human_override_subs[topic]

        def _cb(msg: String, _robot=robot):
            raw = str(msg.data or "").strip()
            active = False
            if raw:
                low = raw.lower()
                if low in ("1", "true", "active", "yes"):
                    active = True
                elif low in ("0", "false", "inactive", "no"):
                    active = False
                else:
                    try:
                        payload = json.loads(raw)
                        active = bool(payload.get("active", False))
                    except Exception:
                        active = False
            if active:
                self._human_override_last_active[_robot] = time.monotonic()

        sub = self.create_subscription(String, topic, _cb, 20)
        self._human_override_subs[topic] = sub
        return sub

    def _is_human_override_active(self, robot: str) -> bool:
        t = float(self._human_override_last_active.get(robot, 0.0))
        if t <= 0.0:
            return False
        return (time.monotonic() - t) <= max(0.05, float(self.human_override_ttl_s))

    def _robot_has_live_heartbeat(self, robot: str) -> bool:
        """
        Require a live heartbeat publisher to treat a robot endpoint as reachable.
        This removes stale DDS action entries after robots power off.
        """
        topic = f"/{robot}/heartbeat"
        try:
            pubs = self.get_publishers_info_by_topic(topic)
        except Exception:
            pubs = []
        return len(pubs) > 0

    def _poll_stop_stdin(self) -> bool:
        """
        Non-blocking stop poll for long-running spin loops.
        Terminal is line-buffered, so user should press Enter after stop token.
        """
        try:
            rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
        except Exception:
            return False
        if not rlist:
            return False
        try:
            raw = sys.stdin.readline()
        except Exception:
            return False
        return self._is_stop_cmd(raw)

    def _poll_stop_keypress(self) -> bool:
        """
        Non-blocking single-key stop poll (no Enter required).

        Requires terminal cbreak mode to be active in the current thread.
        """
        if not sys.stdin.isatty():
            return False
        try:
            rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
        except Exception:
            return False
        if not rlist:
            return False
        try:
            chunk = os.read(sys.stdin.fileno(), 32).decode("utf-8", errors="ignore")
        except Exception:
            return False
        if not chunk:
            return False
        for ch in chunk:
            if ch in ("!", "5", " "):
                return True
        return False

    def _emergency_stop_all(self, show_ui: bool = True):
        """
        Dispatch immediate HOLD to all currently reachable robots.

        This is a best-effort e-stop style software stop for orchestrated motion.
        """
        if self._in_emergency_stop:
            return
        self._in_emergency_stop = True
        try:
            # 1) Cancel active action goals initiated by this orchestrator.
            cancel_futures = []
            for handle in list(self._active_goal_handles.values()):
                try:
                    cancel_futures.append(handle.cancel_goal_async())
                except Exception:
                    pass
            if cancel_futures:
                self._spin_until(
                    lambda: all(f.done() for f in cancel_futures),
                    timeout_sec=0.6,
                    step_sec=0.05,
                )

            # 2) Push zero cmd_vel directly to each reachable robot for immediate brake.
            robots = sorted(self._robots_map().keys())
            if not robots:
                if show_ui:
                    self._print_screen(
                        "STOP ALL ROBOTS",
                        [
                            "No reachable robots found.",
                        ],
                    )
                    self._pause()
                return

            zero = Twist()
            for _ in range(4):
                for robot in robots:
                    try:
                        self._get_or_make_cmd_vel_pub(robot).publish(zero)
                    except Exception:
                        pass
                # Keep ROS pumping briefly between stop bursts.
                rclpy.spin_once(self, timeout_sec=0.03)

            # 3) Also dispatch short HOLD commands as a secondary safety net.
            now = int(time.time())

            def build_goal(robot: str) -> ExecutePlaybook.Goal:
                g = ExecutePlaybook.Goal()
                g.intent_id = f"term_stop_{robot}_{now}"
                g.command_id = "hold"
                g.vehicle_ids = [robot]
                g.parameters_json = json.dumps({"duration_s": 0.3, "speed": 1.0}, separators=(",", ":"))
                return g

            reports = self._dispatch_goal(build_goal, target_robots=robots, allow_interrupt=False)
            if show_ui:
                self._render_reports(reports)
        finally:
            self._in_emergency_stop = False

    def _prompt_float(
        self,
        label: str,
        default: Optional[float] = None,
        allow_zero: bool = True,
    ) -> Optional[float]:
        while True:
            if default is None:
                raw_in = input(f"{label}: ")
                raw = raw_in.strip()
            else:
                raw_in = input(f"{label} [{default}]: ")
                raw = raw_in.strip()
                if raw == "":
                    return float(default)

            if raw.lower() in ("q", "quit", "b", "back"):
                return None
            if self._is_stop_cmd(raw_in):
                self._emergency_stop_all()
                return None
            try:
                val = float(raw)
            except ValueError:
                print("Enter a number.")
                continue

            if not allow_zero and abs(val) < 1e-9:
                print("Value cannot be 0.")
                continue
            return val

    def _emit_json_event(self, topic_type: str, payload: dict):
        """
        Emit fleet-style JSON to terminal and optionally to /fo/task or /fo/audit.
        """
        # Pretty JSON for copy/paste into future fleet_orchestrator integration work.
        raw = json.dumps(payload, indent=2, sort_keys=True)
        print(raw)
        sys.stdout.flush()

        if not self.publish_fleet_topics:
            return

        msg = String()
        msg.data = raw
        if topic_type == "task":
            self.task_pub.publish(msg)
        elif topic_type == "audit":
            self.audit_pub.publish(msg)

    def _resolve_robot_profile_preview(self, robot: str) -> Dict[str, str]:
        out = {
            "drive_type": "unknown",
            "hardware": "unknown",
            "profile_name": "",
            "source": "none",
        }
        if not self._profile_registry:
            return out
        try:
            prof = resolve_robot_profile(self._profile_registry, robot)
            out["drive_type"] = str(prof.get("drive_type") or "unknown")
            out["hardware"] = str(prof.get("hardware") or "unknown")
            out["profile_name"] = str(prof.get("profile_name") or "")
            out["source"] = "registry"
        except Exception:
            pass
        return out

    def _twist_step_text(self, tw) -> str:
        vx = float(getattr(tw.linear, "x", 0.0))
        vy = float(getattr(tw.linear, "y", 0.0))
        wz = float(getattr(tw.angular, "z", 0.0))
        if abs(wz) > 1e-6 and abs(vx) < 1e-6 and abs(vy) < 1e-6:
            return "rotate_left" if wz > 0.0 else "rotate_right"
        if abs(vx) > 1e-6 and abs(vy) < 1e-6 and abs(wz) < 1e-6:
            return "forward" if vx > 0.0 else "backward"
        if abs(vy) > 1e-6 and abs(vx) < 1e-6 and abs(wz) < 1e-6:
            return "strafe_left" if vy > 0.0 else "strafe_right"
        if abs(vx) < 1e-6 and abs(vy) < 1e-6 and abs(wz) < 1e-6:
            return "hold"
        return "mixed_motion"

    def _build_motion_preview(self, robot: str, goal: ExecutePlaybook.Goal) -> Dict[str, object]:
        profile = self._resolve_robot_profile_preview(robot)
        preview: Dict[str, object] = {
            "robot": robot,
            "drive_type": profile["drive_type"],
            "hardware": profile["hardware"],
            "profile_name": profile["profile_name"],
            "profile_source": profile["source"],
            "cardinal_mode": "relative_fallback",
            "strategy_id": "",
            "steps": [],
            "notes": [],
        }
        if goal.command_id != "transit_xy":
            return preview

        try:
            params = json.loads(goal.parameters_json or "{}")
        except Exception:
            params = {}
        speed_scale = max(0.05, min(1.5, float(params.get("speed", 1.0))))
        compiled = compile_transit_xy_plans(
            drive_type=str(profile["drive_type"]),
            hardware=str(profile["hardware"]),
            north_m=float(getattr(goal, "north_m", 0.0)),
            east_m=float(getattr(goal, "east_m", 0.0)),
            v_fwd=self.base_linear_mps * speed_scale,
            v_strafe=self.base_strafe_mps * speed_scale,
            w_rot=self.base_angular_rps * speed_scale,
            cardinal_mode="relative_fallback",
            heading_rad=None,
        )
        preview["strategy_id"] = compiled.strategy_id
        preview["steps"] = [
            {
                "index": i,
                "motion": self._twist_step_text(plan.twist),
                "duration_s": round(float(plan.duration_s), 3),
                "status_text": str(plan.status_text),
            }
            for i, plan in enumerate(compiled.plans, start=1)
        ]
        preview["notes"] = [
            "north/east uses robot body frame without heading sensor",
            "robots with different starting headings diverge in world frame",
        ]
        return preview

    # ---------------- Coordinated XY linear algebra planner ----------------

    def _duration_scales_for_robot(self, robot: Optional[str]) -> Tuple[float, float]:
        """
        Resolve per-robot duration calibration scales.

        Order of precedence:
        1) robot_profiles.yaml per-robot drive params:
           - orchestrator_linear_duration_scale
           - orchestrator_angular_duration_scale
        2) node parameters:
           - linear_duration_scale
           - angular_duration_scale
        """
        linear_scale = float(self.linear_duration_scale)
        angular_scale = float(self.angular_duration_scale)
        if not robot or not self._profile_registry:
            return (
                max(0.1, min(5.0, linear_scale)),
                max(0.1, min(5.0, angular_scale)),
            )
        try:
            prof = resolve_robot_profile(self._profile_registry, robot)
            dp = prof.get("drive_params", {}) or {}
            linear_scale = float(dp.get("orchestrator_linear_duration_scale", linear_scale))
            angular_scale = float(dp.get("orchestrator_angular_duration_scale", angular_scale))
        except Exception:
            pass
        return (
            max(0.1, min(5.0, linear_scale)),
            max(0.1, min(5.0, angular_scale)),
        )

    def _detour_scale_for_robot(self, robot: Optional[str]) -> float:
        """
        Resolve per-robot detour aggressiveness scale for playbook 1.
        1.0 = unchanged, <1.0 = less aggressive V-path detours.
        """
        scale = 1.0
        if robot and self._profile_registry:
            try:
                prof = resolve_robot_profile(self._profile_registry, robot)
                dp = prof.get("drive_params", {}) or {}
                scale = float(dp.get("orchestrator_max_detour_scale", scale))
            except Exception:
                pass
        return max(0.1, min(1.0, scale))

    def _rotation_params_from_delta(
        self, delta_rad: float, speed_scale: float, robot: Optional[str] = None
    ) -> Dict[str, object]:
        """
        Convert a signed rotation delta into executor rotate params.
        """
        direction = "left" if delta_rad >= 0.0 else "right"
        _, angular_scale = self._duration_scales_for_robot(robot)
        duration_s = (
            abs(delta_rad) / max(1e-6, self.base_angular_rps * speed_scale)
        ) * angular_scale
        return {
            "direction": direction,
            "duration_s": duration_s,
            "speed": speed_scale,
        }

    def _transit_params_from_length(
        self, length_m: float, speed_scale: float, robot: Optional[str] = None
    ) -> Dict[str, object]:
        """
        Convert a signed path length into executor transit params.
        """
        direction = "forward" if length_m >= 0.0 else "backward"
        linear_scale, _ = self._duration_scales_for_robot(robot)
        duration_s = (
            abs(length_m) / max(1e-6, self.base_linear_mps * speed_scale)
        ) * linear_scale
        return {
            "direction": direction,
            "duration_s": duration_s,
            "speed": speed_scale,
        }

    def _line_detour_distance(self, waypoint: Tuple[float, float], goal_xy: Tuple[float, float]) -> float:
        """
        Max lateral deviation from the straight line start->goal is the perpendicular
        distance from the intermediate waypoint to that line.
        """
        wx, wy = waypoint
        gx, gy = goal_xy
        norm = math.hypot(gx, gy)
        if norm < 1e-9:
            return 0.0
        # 2D cross-product magnitude divided by |goal|.
        return abs(wx * gy - wy * gx) / norm

    def _solve_two_leg_lengths(self, dx: float, dy: float, theta1: float, theta2: float) -> Optional[Tuple[float, float]]:
        """
        Solve the 2x2 linear system:
          l1*[cos(theta1), sin(theta1)] + l2*[cos(theta2), sin(theta2)] = [dx, dy]

        This is the core linear algebra step that transforms two heading choices into
        two segment lengths that exactly reach the destination vector.
        """
        a11, a12 = math.cos(theta1), math.cos(theta2)
        a21, a22 = math.sin(theta1), math.sin(theta2)
        det = a11 * a22 - a12 * a21
        if abs(det) < 1e-6:
            return None
        inv11, inv12 = a22 / det, -a12 / det
        inv21, inv22 = -a21 / det, a11 / det
        l1 = inv11 * dx + inv12 * dy
        l2 = inv21 * dx + inv22 * dy
        return (l1, l2)

    def _signed_distance_to_line(self, point_xy: Tuple[float, float], line_dir_xy: Tuple[float, float]) -> float:
        """
        Signed perpendicular distance from point to the infinite line through origin
        with direction line_dir_xy.
        """
        px, py = point_xy
        lx, ly = line_dir_xy
        denom = math.hypot(lx, ly)
        if denom < 1e-9:
            return 0.0
        return (px * ly - py * lx) / denom

    def _rotate_vector_into_robot_frame(
        self, x_main: float, y_main: float, robot_heading_from_main_rad: float
    ) -> Tuple[float, float]:
        """
        Convert a vector expressed in main-robot frame into a non-main robot frame.

        robot_heading_from_main_rad:
        - 0 rad means robot faces same direction as main.
        - Positive follows this module's clock conversion convention.
        """
        a = float(robot_heading_from_main_rad)
        c = math.cos(a)
        s = math.sin(a)
        # v_main = R(a) * v_robot  =>  v_robot = R(-a) * v_main
        xr = float(x_main) * c + float(y_main) * s
        yr = -float(x_main) * s + float(y_main) * c
        return xr, yr

    def _plan_two_leg_deterministic(
        self,
        dx: float,
        dy: float,
        turn_side: str,
        max_detour_m: float,
        theta2_fixed: float,
        target_max_detour_m: float,
        ref_line_dir_xy: Tuple[float, float],
        max_samples: int = 120,
    ) -> Optional[TwoLegPlan]:
        """
        Deterministic two-leg planner using constrained linear algebra search.

        We keep a fixed final heading theta2_fixed, then scan theta1 on the
        requested side and solve segment lengths from:
          l1*u(theta1) + l2*u(theta2) = [dx, dy]

        Candidate quality is based on closeness to target_max_detour_m while enforcing:
        - forward+forward legs (l1,l2 > 0),
        - max detour bound.
        """
        side_sign = +1.0 if turn_side == "left" else -1.0
        min_abs = math.radians(4.0)
        # Wider heading envelope improves feasibility for large lateral offsets.
        max_abs = math.radians(85.0)
        theta2 = float(theta2_fixed)

        best: Optional[TwoLegPlan] = None
        best_err = float("inf")

        for i in range(max_samples):
            ratio = i / max(1, max_samples - 1)
            h1 = min_abs + ratio * (max_abs - min_abs)
            theta1 = side_sign * h1

            solved = self._solve_two_leg_lengths(dx=dx, dy=dy, theta1=theta1, theta2=theta2)
            if solved is None:
                continue
            l1, l2 = solved
            # Enforce forward+forward style.
            if l1 <= 0.02 or l2 <= 0.02:
                continue

            wx = l1 * math.cos(theta1)
            wy = l1 * math.sin(theta1)
            d_waypoint = abs(self._signed_distance_to_line((wx, wy), ref_line_dir_xy))
            d_goal = abs(self._signed_distance_to_line((dx, dy), ref_line_dir_xy))
            # For a polyline, max distance from reference line occurs at segment endpoints.
            achieved = max(d_waypoint, d_goal)
            if achieved > max_detour_m + 1e-9:
                continue

            err = abs(achieved - target_max_detour_m)
            if err < best_err:
                best_err = err
                best = TwoLegPlan(
                theta1=theta1,
                theta2=theta2,
                length1_m=l1,
                length2_m=l2,
                max_detour_m=achieved,
            )
        return best

    def _plan_two_leg_vector_sum(
        self,
        dx: float,
        dy: float,
        turn_side: str,
        target_detour_m: float,
        max_detour_m: float,
        ref_line_dir_xy: Tuple[float, float],
    ) -> TwoLegPlan:
        """
        Closed-form 2-vector decomposition for V-path motion.

        We construct two vectors that sum exactly to the destination vector:
          v1 + v2 = d
        with symmetric headings around destination direction:
          theta1 = theta_goal + s*alpha
          theta2 = theta_goal - s*alpha

        This guarantees a valid decomposition with positive lengths:
          l1 = l2 = |d| / (2*cos(alpha))
        for alpha in [0, pi/2).
        """
        d = math.hypot(dx, dy)
        if d < 1e-9:
            return TwoLegPlan(
                theta1=0.0,
                theta2=0.0,
                length1_m=0.0,
                length2_m=0.0,
                max_detour_m=0.0,
            )

        theta_goal = math.atan2(dy, dx)
        side_sign = +1.0 if turn_side == "left" else -1.0

        # Desired geometric detour from straight destination line.
        # For symmetric V, max perpendicular distance h = (|d|/2)*tan(alpha).
        h = max(0.0, min(float(target_detour_m), float(max_detour_m)))
        alpha = math.atan2(2.0 * h, d) if h > 1e-9 else 0.0
        alpha = min(alpha, math.radians(85.0))

        c = max(1e-6, math.cos(alpha))
        l = d / (2.0 * c)
        theta1 = theta_goal + side_sign * alpha
        theta2 = theta_goal - side_sign * alpha

        wx = l * math.cos(theta1)
        wy = l * math.sin(theta1)
        d_waypoint = abs(self._signed_distance_to_line((wx, wy), ref_line_dir_xy))
        d_goal = abs(self._signed_distance_to_line((dx, dy), ref_line_dir_xy))
        achieved = max(d_waypoint, d_goal)

        return TwoLegPlan(
            theta1=theta1,
            theta2=theta2,
            length1_m=l,
            length2_m=l,
            max_detour_m=achieved,
        )

    def _robot_goal_vectors_for_formation(
        self,
        robots: List[str],
        anchor_robot: str,
        x_m: float,
        y_m: float,
        spacing_m: float,
        formation_dir_rad: float,
    ) -> Dict[str, Tuple[float, float]]:
        """
        Compute per-robot destination vectors in anchor-relative formation terms.

        Assumption (explicitly surfaced to user in UI/docs):
        robots begin in a line referenced to anchor_robot with spacing spacing_m
        and direction formation_dir_rad in the same body frame used by x/y.

        Linear algebra simplification:
        - Let v = [x, y] be anchor destination vector.
        - Let s = spacing * [cos(phi), sin(phi)] be formation axis vector.
        - Robot k (relative index from anchor) gets v + k*s.
        """
        ordered = list(robots)
        if anchor_robot not in ordered:
            return {r: (x_m, y_m) for r in ordered}

        anchor_idx = ordered.index(anchor_robot)
        sx = spacing_m * math.cos(formation_dir_rad)
        sy = spacing_m * math.sin(formation_dir_rad)

        out: Dict[str, Tuple[float, float]] = {}
        for idx, robot in enumerate(ordered):
            k = idx - anchor_idx
            out[robot] = (x_m + k * sx, y_m + k * sy)
        return out

    def _clock_to_rad(self, clock_value: float) -> float:
        """
        Convert clock notation to robot-relative radians.
        Conventions used by this tool:
        - 12 o'clock -> 0 deg (forward, +x)
        - 3 o'clock  -> 90 deg (right, +y)
        - 6 o'clock  -> 180 deg (backward, -x)
        - 9 o'clock  -> 270 deg (left, -y)
        """
        normalized = float(clock_value) % 12.0
        return math.radians(normalized * 30.0)

    def _default_heading_config(
        self,
        detected_robots: List[str],
        main_robot: str,
    ) -> Dict[str, Dict[str, float]]:
        """
        Default heading assumptions for non-main robots.
        - 12 o'clock means same heading as main robot.
        """
        out: Dict[str, Dict[str, float]] = {}
        for robot in detected_robots:
            if robot == main_robot:
                continue
            out[robot] = {"heading_clock": 12.0}
        return out

    def _prompt_heading_config(
        self,
        detected_robots: List[str],
        main_robot: str,
        defaults: Dict[str, Dict[str, float]],
    ) -> Optional[Dict[str, Dict[str, float]]]:
        """
        Ask whether all robots start with same heading as main, or capture
        per-robot heading clock values.
        """
        lines = [
            "Initial heading config:",
            f"Main: {main_robot}",
            "All robots same heading?",
            "(12 o'clock means yes)",
            "",
            "Use defaults? [Y/n]",
        ]
        self._print_screen("HEADING CONFIG", lines)
        use_defaults = input("> ").strip().lower()
        if use_defaults in ("", "y", "yes"):
            return defaults
        if use_defaults in ("b", "back", "q", "quit", "0"):
            return None

        custom: Dict[str, Dict[str, float]] = {}
        for robot in detected_robots:
            if robot == main_robot:
                continue
            h_default = float(defaults.get(robot, {}).get("heading_clock", 12.0))
            self._print_screen(
                "ROBOT HEADING",
                [
                    f"relative to the main robot ({main_robot}),",
                    f"{robot} is pointing toward ___ o'clock",
                    "enter a value:",
                    "",
                    "12 -> same direction as main",
                    "3 -> right of main heading",
                    "6 -> opposite direction",
                    "9 -> left of main heading",
                    "Type 'back' to cancel.",
                ],
            )
            heading_clock = self._prompt_float(
                f"{robot} heading o'clock", default=h_default, allow_zero=False
            )
            if heading_clock is None:
                return None
            custom[robot] = {"heading_clock": float(heading_clock) % 12.0}
        return custom

    # ---------------- ROS discovery/action ----------------

    def _discover_action_servers(self) -> List[Tuple[str, str]]:
        """
        Returns list of (robot_name, endpoint).

        Endpoint can be:
        - /<robot>/execute_playbook      (action transport)
        - /<robot>/execute_playbook_cmd  (standalone topic transport)
        """
        found: Dict[str, str] = {}

        if HAS_FLEET_ACTION:
            # Primary path: native action graph API.
            try:
                action_names_and_types = self.get_action_names_and_types()
            except Exception:
                action_names_and_types = []

            for action_name, types in action_names_and_types:
                if ACTION_TYPE not in types:
                    continue
                m = ACTION_RE.match(action_name)
                if not m:
                    continue
                found[m.group(1)] = action_name

            # Fallback path: discover action servers via send_goal service endpoints.
            if not found:
                try:
                    services_and_types = self.get_service_names_and_types()
                except Exception:
                    services_and_types = []
                for service_name, types in services_and_types:
                    if ACTION_SEND_GOAL_TYPE not in types:
                        continue
                    m = ACTION_SEND_GOAL_SVC_RE.match(service_name)
                    if not m:
                        continue
                    robot = m.group(1)
                    found[robot] = f"/{robot}/execute_playbook"

        # Standalone transport fallback via topic discovery.
        try:
            topic_names_and_types = self.get_topic_names_and_types()
        except Exception:
            topic_names_and_types = []
        for topic_name, types in topic_names_and_types:
            if CMD_TOPIC_TYPE not in types:
                continue
            m = CMD_TOPIC_RE.match(topic_name)
            if not m:
                continue
            robot = m.group(1)
            # Prefer action endpoint if already found.
            found.setdefault(robot, topic_name)

        pairs = sorted(found.items(), key=lambda p: p[0])
        return pairs

    def _reachable_robots_map(self, probe_timeout_s: Optional[float] = None) -> Dict[str, str]:
        """
        Return only robots whose playbook endpoint is currently reachable.
        """
        discovered = {robot: endpoint for robot, endpoint in self._discover_action_servers()}
        if not discovered:
            return {}

        timeout_s = (
            self.reachable_probe_timeout_s
            if probe_timeout_s is None
            else max(0.0, float(probe_timeout_s))
        )
        # Standalone topic visibility map used as a robust fallback when
        # stale action graph entries exist for a robot.
        topic_transport: Dict[str, str] = {}
        try:
            topic_names_and_types = self.get_topic_names_and_types()
        except Exception:
            topic_names_and_types = []
        for topic_name, types in topic_names_and_types:
            if CMD_TOPIC_TYPE not in types:
                continue
            m = CMD_TOPIC_RE.match(topic_name)
            if not m:
                continue
            topic_transport[m.group(1)] = topic_name

        reachable: Dict[str, str] = {}
        for robot, endpoint in discovered.items():
            if not self._robot_has_live_heartbeat(robot):
                continue
            if endpoint.endswith("/execute_playbook_cmd"):
                reachable[robot] = endpoint
                continue
            client = self._get_or_make_client(endpoint)
            try:
                if client.wait_for_server(timeout_sec=timeout_s):
                    reachable[robot] = endpoint
                elif robot in topic_transport:
                    # Action endpoint exists in graph but is stale/unreachable.
                    # Fall back to standalone topic transport for that robot.
                    reachable[robot] = topic_transport[robot]
            except Exception:
                if robot in topic_transport:
                    reachable[robot] = topic_transport[robot]
        return reachable

    def _robots_map(self) -> Dict[str, str]:
        robots = self._reachable_robots_map()
        for robot in robots.keys():
            self._ensure_human_override_sub(robot)
        return robots

    def _get_or_make_client(self, action_name: str) -> ActionClient:
        if not HAS_FLEET_ACTION:
            raise RuntimeError("Action interface not available in standalone mode")
        client = self._action_clients.get(action_name)
        if client is None:
            client = ActionClient(self, ExecutePlaybook, action_name)
            self._action_clients[action_name] = client
        return client

    def _send_goal(self, robot: str, action_name: str, goal: ExecutePlaybook.Goal) -> GoalReport:
        if action_name.endswith("/execute_playbook_cmd"):
            intent_id = str(goal.intent_id)
            self._ensure_playbook_result_sub(robot)
            # Clear stale result for same intent if any.
            self._topic_results.pop((robot, intent_id), None)
            payload = {
                "intent_id": intent_id,
                "command_id": str(goal.command_id),
                "vehicle_ids": list(getattr(goal, "vehicle_ids", [robot])),
                "parameters_json": str(getattr(goal, "parameters_json", "")),
                "north_m": float(getattr(goal, "north_m", 0.0)),
                "east_m": float(getattr(goal, "east_m", 0.0)),
            }
            msg = String()
            msg.data = json.dumps(payload, separators=(",", ":"))
            try:
                self._get_or_make_playbook_cmd_pub(robot).publish(msg)
            except Exception as ex:
                return GoalReport(
                    robot=robot,
                    accepted=False,
                    success=False,
                    reason=f"topic dispatch failed: {ex}",
                )
            # Wait for explicit robot-side completion result to avoid queuing
            # unchecked commands that can overlap and destabilize motion.
            done = self._spin_until(
                lambda: (robot, intent_id) in self._topic_results,
                timeout_sec=90.0,
            )
            if not done:
                return GoalReport(
                    robot=robot,
                    accepted=True,
                    success=False,
                    reason="topic result timeout",
                )
            return self._topic_results.pop(
                (robot, intent_id),
                GoalReport(robot=robot, accepted=True, success=False, reason="missing topic result"),
            )

        client = self._get_or_make_client(action_name)
        if not client.wait_for_server(timeout_sec=2.0):
            return GoalReport(robot=robot, accepted=False, success=False, reason="server unavailable")

        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        if not send_future.done():
            return GoalReport(robot=robot, accepted=False, success=False, reason="goal send timeout")

        goal_handle = send_future.result()
        if goal_handle is None:
            return GoalReport(robot=robot, accepted=False, success=False, reason="no goal handle")
        if not goal_handle.accepted:
            return GoalReport(robot=robot, accepted=False, success=False, reason="goal rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=90.0)
        if not result_future.done():
            return GoalReport(robot=robot, accepted=True, success=False, reason="result timeout")

        wrapped = result_future.result()
        result_msg = wrapped.result if wrapped is not None else None
        if result_msg is None:
            return GoalReport(robot=robot, accepted=True, success=False, reason="empty result")

        success = bool(getattr(result_msg, "success", True))
        reason = str(getattr(result_msg, "reason", "")).strip() or ("ok" if success else "failed")
        accepted = bool(getattr(result_msg, "accepted", True))
        return GoalReport(robot=robot, accepted=accepted, success=success, reason=reason)

    def _target_actions(self, robots_map: Dict[str, str]) -> List[Tuple[str, str]]:
        if self.selected_robot:
            action = robots_map.get(self.selected_robot)
            if action:
                return [(self.selected_robot, action)]
            return []
        return [(r, a) for r, a in robots_map.items()]

    def _spin_until(self, done_fn, timeout_sec: float, step_sec: float = 0.05) -> bool:
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        fd = None
        old_tty = None
        cbreak_on = False
        if (not self._in_emergency_stop) and sys.stdin.isatty():
            try:
                fd = sys.stdin.fileno()
                old_tty = termios.tcgetattr(fd)
                tty.setcbreak(fd)
                cbreak_on = True
            except Exception:
                cbreak_on = False
        try:
            while rclpy.ok():
                if not self._in_emergency_stop:
                    # Immediate one-key stop path first.
                    if self._poll_stop_keypress() or self._poll_stop_stdin():
                        self._emergency_stop_all(show_ui=False)
                        return done_fn()
                if done_fn():
                    return True
                now = time.monotonic()
                if now >= deadline:
                    return done_fn()
                rclpy.spin_once(self, timeout_sec=min(step_sec, deadline - now))
            return done_fn()
        finally:
            if cbreak_on and fd is not None and old_tty is not None:
                try:
                    termios.tcsetattr(fd, termios.TCSADRAIN, old_tty)
                except Exception:
                    pass

    def _dispatch_goal(
        self,
        goal_factory,
        target_robots: Optional[List[str]] = None,
        allow_interrupt: bool = True,
    ) -> List[GoalReport]:
        robots_map = self._robots_map()
        if target_robots is None:
            targets = self._target_actions(robots_map)
        else:
            wanted = {str(r) for r in target_robots}
            targets = [(r, a) for r, a in robots_map.items() if r in wanted]
        reports: List[GoalReport] = []

        if not targets:
            return reports

        pending: Dict[str, Dict[str, object]] = {}
        pending_topic: Dict[str, Dict[str, str]] = {}

        # Phase 1: send all goals first so robots can start at roughly the same time.
        for robot, action_name in targets:
            if not action_name.endswith("/execute_playbook_cmd"):
                client = self._get_or_make_client(action_name)
                if not client.wait_for_server(timeout_sec=self.dispatch_timeout_s):
                    reason = (
                        f"skipped due to timeout: waited > {self.dispatch_timeout_s:.1f}s "
                        "for action server"
                    )
                    self._emit_json_event(
                        "audit",
                        {
                            "type": "audit_event",
                            "stage": "execute_playbook_dispatch",
                            "intent_id": "",
                            "robot": robot,
                            "status": "skipped",
                            "details": reason,
                        },
                    )
                    reports.append(
                        GoalReport(robot=robot, accepted=False, success=False, reason=reason)
                    )
                    continue
            goal = goal_factory(robot)
            preview = self._build_motion_preview(robot, goal)
            dispatch_task = {
                "type": "playbook_command",
                "intent_id": str(goal.intent_id),
                "command_id": str(goal.command_id),
                "platform": {
                    "adapter_type": "terminal_orchestrator",
                    "platform_family": "robot_legion_teleop_python",
                },
                "parameters": json.loads(goal.parameters_json or "{}") if goal.parameters_json else {},
                "targets": robot,
                "trace": {
                    "translator": "terminal_orchestrator",
                    "confidence": 1.0,
                    "explanation": "manual terminal playbook dispatch",
                },
                "preview": preview,
            }
            self._emit_json_event("task", dispatch_task)
            if action_name.endswith("/execute_playbook_cmd"):
                intent_id = str(goal.intent_id)
                self._ensure_playbook_result_sub(robot)
                # Drop any stale result with same intent id before dispatch.
                self._topic_results.pop((robot, intent_id), None)
                payload = {
                    "intent_id": intent_id,
                    "command_id": str(goal.command_id),
                    "vehicle_ids": list(getattr(goal, "vehicle_ids", [robot])),
                    "parameters_json": str(getattr(goal, "parameters_json", "")),
                    "north_m": float(getattr(goal, "north_m", 0.0)),
                    "east_m": float(getattr(goal, "east_m", 0.0)),
                }
                msg = String()
                msg.data = json.dumps(payload, separators=(",", ":"))
                try:
                    self._get_or_make_playbook_cmd_pub(robot).publish(msg)
                except Exception as ex:
                    reason = f"topic dispatch failed: {ex}"
                    self._emit_json_event(
                        "audit",
                        {
                            "type": "audit_event",
                            "stage": "execute_playbook_dispatch",
                            "intent_id": intent_id,
                            "robot": robot,
                            "status": "failed",
                            "details": reason,
                        },
                    )
                    reports.append(
                        GoalReport(robot=robot, accepted=False, success=False, reason=reason)
                    )
                    continue
                pending_topic[robot] = {"intent_id": intent_id}
                self._emit_json_event(
                    "audit",
                    {
                        "type": "audit_event",
                        "stage": "execute_playbook_dispatch",
                        "intent_id": intent_id,
                        "robot": robot,
                        "status": "dispatched",
                        "details": "topic dispatched",
                    },
                )
            else:
                send_future = client.send_goal_async(goal)
                pending[robot] = {"send_future": send_future, "intent_id": str(goal.intent_id)}

        if not pending and not pending_topic:
            return reports

        # Wait for send_goal responses.
        self._spin_until(
            lambda: all(state["send_future"].done() for state in pending.values()),
            timeout_sec=self.send_goal_response_timeout_s,
        )

        result_pending: Dict[str, Dict[str, object]] = {}
        for robot, state in pending.items():
            send_future = state["send_future"]
            if not send_future.done():
                reason = (
                    f"skipped due to timeout: waited > {self.send_goal_response_timeout_s:.1f}s "
                    "for send_goal response"
                )
                self._emit_json_event(
                    "audit",
                    {
                        "type": "audit_event",
                        "stage": "execute_playbook_dispatch",
                        "intent_id": str(state.get("intent_id", "")),
                        "robot": robot,
                        "status": "skipped",
                        "details": reason,
                    },
                )
                reports.append(
                    GoalReport(robot=robot, accepted=False, success=False, reason=reason)
                )
                continue

            goal_handle = send_future.result()
            if goal_handle is None:
                reports.append(
                    GoalReport(robot=robot, accepted=False, success=False, reason="no goal handle")
                )
                continue
            if not goal_handle.accepted:
                reports.append(
                    GoalReport(robot=robot, accepted=False, success=False, reason="goal rejected")
                )
                continue

            self._active_goal_handles[robot] = goal_handle
            result_pending[robot] = {
                "future": goal_handle.get_result_async(),
                "intent_id": state.get("intent_id", ""),
            }

        if result_pending or pending_topic:
            self._spin_until(
                lambda: (
                    all(state["future"].done() for state in result_pending.values())
                    and all(
                        (robot, str(state["intent_id"])) in self._topic_results
                        for robot, state in pending_topic.items()
                    )
                ),
                timeout_sec=90.0,
            )

        for robot, state in result_pending.items():
            result_future = state["future"]
            intent_id = str(state.get("intent_id", ""))
            if not result_future.done():
                self._emit_json_event(
                    "audit",
                    {
                        "type": "audit_event",
                        "stage": "execute_playbook_dispatch",
                        "intent_id": intent_id,
                        "robot": robot,
                        "status": "failed",
                        "details": "result timeout",
                    },
                )
                reports.append(
                    GoalReport(robot=robot, accepted=True, success=False, reason="result timeout")
                )
                self._active_goal_handles.pop(robot, None)
                continue

            wrapped = result_future.result()
            result_msg = wrapped.result if wrapped is not None else None
            if result_msg is None:
                self._emit_json_event(
                    "audit",
                    {
                        "type": "audit_event",
                        "stage": "execute_playbook_dispatch",
                        "intent_id": intent_id,
                        "robot": robot,
                        "status": "failed",
                        "details": "empty result",
                    },
                )
                reports.append(
                    GoalReport(robot=robot, accepted=True, success=False, reason="empty result")
                )
                self._active_goal_handles.pop(robot, None)
                continue

            success = bool(getattr(result_msg, "success", True))
            reason = str(getattr(result_msg, "reason", "")).strip() or ("ok" if success else "failed")
            accepted = bool(getattr(result_msg, "accepted", True))
            self._emit_json_event(
                "audit",
                    {
                        "type": "audit_event",
                        "stage": "execute_playbook_dispatch",
                        "intent_id": intent_id,
                        "robot": robot,
                        "status": "succeeded" if success else "failed",
                    "accepted": accepted,
                    "reason": reason,
                },
            )
            reports.append(
                GoalReport(robot=robot, accepted=accepted, success=success, reason=reason)
            )
            self._active_goal_handles.pop(robot, None)

        for robot, state in pending_topic.items():
            intent_id = str(state.get("intent_id", ""))
            rep = self._topic_results.pop((robot, intent_id), None)
            if rep is None:
                reason = "topic result timeout"
                self._emit_json_event(
                    "audit",
                    {
                        "type": "audit_event",
                        "stage": "execute_playbook_dispatch",
                        "intent_id": intent_id,
                        "robot": robot,
                        "status": "failed",
                        "details": reason,
                    },
                )
                reports.append(
                    GoalReport(robot=robot, accepted=True, success=False, reason=reason)
                )
                continue

            status = "succeeded" if (rep.accepted and rep.success) else "failed"
            self._emit_json_event(
                "audit",
                {
                    "type": "audit_event",
                    "stage": "execute_playbook_dispatch",
                    "intent_id": intent_id,
                    "robot": robot,
                    "status": status,
                    "details": rep.reason,
                },
            )
            reports.append(rep)

        return reports

    # ---------------- Menus ----------------

    def _target_menu(self):
        while rclpy.ok():
            robots_map = self._robots_map()
            robots = sorted(robots_map.keys())
            lines = [
                "Select target robots",
                "",
                "a) all robots",
            ]
            for i, robot in enumerate(robots, start=1):
                lines.append(f"{i}) {robot}")
            lines.extend(
                [
                    "",
                    "!) STOP ALL ROBOTS",
                    "b) back",
                ]
            )
            self._print_screen("TARGET SELECT", lines)

            raw_choice = input("Choice: ")
            choice = raw_choice.strip().lower()
            if self._is_stop_cmd(raw_choice):
                self._emergency_stop_all()
                continue
            if choice in ("b", "back", "q", "quit"):
                return
            if choice == "a":
                self.selected_robot = None
                return
            try:
                idx = int(choice)
            except ValueError:
                continue
            if 1 <= idx <= len(robots):
                self.selected_robot = robots[idx - 1]
                return

    def _render_reports(self, reports: List[GoalReport], pause: bool = True):
        lines = ["Execution report", ""]
        if not reports:
            lines.extend(["No target robots found."])
        else:
            for rep in reports:
                if rep.accepted and rep.success:
                    lines.append(f"{rep.robot}: OK")
                else:
                    lines.append(f"{rep.robot}: FAIL")
                    lines.append(f"  {rep.reason}")
        self._print_screen("RESULT", lines)
        if pause:
            self._pause()

    def _confirm(self, summary_lines: List[str]) -> bool:
        self._print_screen(
            "EXECUTION SUMMARY",
            summary_lines
            + [
                "",
                "EXECUTE? y/n",
                "STOP ALL: ! / stop",
                "or space / 5",
            ],
        )
        raw_answer = input("> ")
        answer = raw_answer.strip().lower()
        if self._is_stop_cmd(raw_answer):
            self._emergency_stop_all()
            return False
        return answer in ("y", "yes")

    # ---------------- Playbook sequence submenu ----------------

    def _collect_sequence_entry(
        self, choice: str, existing_entry: Optional[Dict[str, Any]] = None
    ) -> Optional[Dict[str, Any]]:
        """
        Build a queued sequence entry for the selected playbook.

        Notes:
        - Playbook 1 remains fully interactive at run time because it has a large
          planning form (main robot, relative config, heading clocks, etc).
        - Playbooks 2/3/4 capture variables up front and execute later.
        """
        existing_params = dict((existing_entry or {}).get("params", {}) or {})

        if choice == "1":
            detected_robots = sorted(self._robots_map().keys())
            if not detected_robots:
                self._print_screen(
                    "MOVE XY REQUIREMENT",
                    [
                        "No detected robots.",
                        "Start robot bringup and retry.",
                    ],
                )
                self._pause()
                return None

            x_default = float(existing_params.get("x_m", 1.0))
            y_default = float(existing_params.get("y_m", 1.0))
            speed_default = float(existing_params.get("speed", self.default_speed_scale))
            max_detour_default = float(existing_params.get("max_detour_m", 0.8))

            x_m = self._prompt_float("x meters", default=x_default)
            if x_m is None:
                return None
            y_m = self._prompt_float("y meters", default=y_default)
            if y_m is None:
                return None
            speed = self._prompt_float("speed scale", default=speed_default, allow_zero=False)
            if speed is None:
                return None
            speed = max(0.05, min(1.5, float(speed)))
            max_detour_m = self._prompt_float(
                "MAX PATH OFFSET m", default=max_detour_default, allow_zero=False
            )
            if max_detour_m is None:
                return None
            max_detour_m = max(0.05, float(max_detour_m))

            existing_main_robot = str(existing_params.get("main_robot", "")).strip()
            default_main_idx = 1
            if existing_main_robot in detected_robots:
                default_main_idx = detected_robots.index(existing_main_robot) + 1
            self._print_screen(
                "MAIN ROBOT SELECT",
                [
                    "Choose main robot (anchor)",
                    "Formation offsets are",
                    "interpreted from this robot.",
                    "",
                ] + [f"{i+1}) {r}" for i, r in enumerate(detected_robots)],
            )
            while True:
                main_choice = input(f"Main robot index [{default_main_idx}]: ").strip()
                if main_choice == "":
                    main_idx = default_main_idx
                    break
                if main_choice.lower() in ("b", "back", "q", "quit", "0"):
                    return None
                try:
                    parsed = int(main_choice)
                except ValueError:
                    print("Invalid index. Auto-selecting 1.")
                    main_idx = 1
                    break
                if 1 <= parsed <= len(detected_robots):
                    main_idx = parsed
                    break
                print("Invalid index. Auto-selecting 1.")
                main_idx = 1
                break
            main_robot = detected_robots[main_idx - 1]

            default_head = self._default_heading_config(
                detected_robots=detected_robots,
                main_robot=main_robot,
            )
            existing_heading_cfg = dict(existing_params.get("heading_cfg", {}) or {})
            for robot in detected_robots:
                if robot == main_robot:
                    continue
                if robot in existing_heading_cfg:
                    try:
                        h = float(existing_heading_cfg[robot].get("heading_clock", 12.0))
                    except Exception:
                        h = 12.0
                    default_head[robot] = {"heading_clock": h % 12.0}
            heading_cfg = self._prompt_heading_config(
                detected_robots=detected_robots,
                main_robot=main_robot,
                defaults=default_head,
            )
            if heading_cfg is None:
                return None

            return {
                "playbook": "move_xy",
                "label": "Move to objective",
                "params": {
                    "detected_robots": detected_robots,
                    "x_m": float(x_m),
                    "y_m": float(y_m),
                    "speed": float(speed),
                    "max_detour_m": float(max_detour_m),
                    "main_robot": str(main_robot),
                    "heading_cfg": heading_cfg,
                },
                "deferred_input": False,
            }
        if choice == "2":
            speed_default = float(existing_params.get("speed", 1.0))
            speed = self._prompt_float("speed scale", default=speed_default, allow_zero=False)
            if speed is None:
                return None
            speed = max(0.05, min(1.5, float(speed)))
            return {
                "playbook": "execute_all_commands",
                "label": "Execute ALL commands",
                "params": {"speed": speed},
                "deferred_input": False,
            }
        if choice == "3":
            meters_default = float(existing_params.get("meters", 1.0))
            speed_default = float(existing_params.get("speed", self.default_speed_scale))
            meters = self._prompt_float("distance meters", default=meters_default, allow_zero=False)
            if meters is None:
                return None
            speed = self._prompt_float("speed scale", default=speed_default, allow_zero=False)
            if speed is None:
                return None
            speed = max(0.05, min(1.5, float(speed)))
            return {
                "playbook": "transit_distance",
                "label": "Transit distance",
                "params": {"meters": float(meters), "speed": speed},
                "deferred_input": False,
            }
        if choice == "4":
            degrees_default = float(existing_params.get("degrees", 45.0))
            speed_default = float(existing_params.get("speed", self.default_speed_scale))
            degrees = self._prompt_float("degrees", default=degrees_default, allow_zero=False)
            if degrees is None:
                return None
            speed = self._prompt_float("speed scale", default=speed_default, allow_zero=False)
            if speed is None:
                return None
            speed = max(0.05, min(1.5, float(speed)))
            return {
                "playbook": "rotate_degrees",
                "label": "Rotate degrees",
                "params": {"degrees": float(degrees), "speed": speed},
                "deferred_input": False,
            }
        return None

    def _sequence_choice_for_entry(self, entry: Dict[str, Any]) -> Optional[str]:
        playbook = str(entry.get("playbook", ""))
        if playbook == "move_xy":
            return "1"
        if playbook == "execute_all_commands":
            return "2"
        if playbook == "transit_distance":
            return "3"
        if playbook == "rotate_degrees":
            return "4"
        return None

    def _normalize_sequence_entry_from_json_step(
        self, step: Dict[str, Any], index: int
    ) -> Tuple[Optional[Dict[str, Any]], Optional[str]]:
        """
        Normalize one JSON sequence step into the internal queue entry format.

        Accepted input shape (fleet-ready):
          {"playbook": "...", "label": "...", "params": {...}}
        """
        playbook = str(step.get("playbook", "")).strip()
        label = str(step.get("label", "")).strip()
        params = dict(step.get("params", {}) or {})

        if playbook not in (
            "move_xy",
            "execute_all_commands",
            "transit_distance",
            "rotate_degrees",
        ):
            return None, f"step {index}: unsupported playbook '{playbook}'"

        if playbook == "move_xy":
            try:
                x_m = float(params.get("x_m", 0.0))
                y_m = float(params.get("y_m", 0.0))
                speed = max(0.05, min(1.5, float(params.get("speed", self.default_speed_scale))))
                max_detour_m = max(0.05, float(params.get("max_detour_m", 0.8)))
            except Exception:
                return None, f"step {index}: invalid move_xy numeric params"

            main_robot = str(params.get("main_robot", "")).strip()
            if not main_robot:
                return None, f"step {index}: move_xy requires non-empty params.main_robot"

            heading_cfg_in = dict(params.get("heading_cfg", {}) or {})
            heading_cfg: Dict[str, Dict[str, float]] = {}
            for robot, cfg in heading_cfg_in.items():
                try:
                    heading_clock = float(dict(cfg or {}).get("heading_clock", 12.0)) % 12.0
                except Exception:
                    heading_clock = 12.0
                heading_cfg[str(robot)] = {"heading_clock": heading_clock}

            detected_robots = [str(r) for r in list(params.get("detected_robots", []))]

            return (
                {
                    "playbook": "move_xy",
                    "label": label or "Move to objective",
                    "params": {
                        "detected_robots": detected_robots,
                        "x_m": x_m,
                        "y_m": y_m,
                        "speed": speed,
                        "max_detour_m": max_detour_m,
                        "main_robot": main_robot,
                        "heading_cfg": heading_cfg,
                    },
                    "deferred_input": False,
                },
                None,
            )

        if playbook == "execute_all_commands":
            try:
                speed = max(0.05, min(1.5, float(params.get("speed", 1.0))))
            except Exception:
                return None, f"step {index}: invalid execute_all_commands speed"
            return (
                {
                    "playbook": "execute_all_commands",
                    "label": label or "Execute ALL commands",
                    "params": {"speed": speed},
                    "deferred_input": False,
                },
                None,
            )

        if playbook == "transit_distance":
            try:
                meters = float(params.get("meters", 1.0))
                speed = max(0.05, min(1.5, float(params.get("speed", self.default_speed_scale))))
            except Exception:
                return None, f"step {index}: invalid transit_distance params"
            return (
                {
                    "playbook": "transit_distance",
                    "label": label or "Transit distance",
                    "params": {"meters": meters, "speed": speed},
                    "deferred_input": False,
                },
                None,
            )

        # rotate_degrees
        try:
            degrees = float(params.get("degrees", 45.0))
            speed = max(0.05, min(1.5, float(params.get("speed", self.default_speed_scale))))
        except Exception:
            return None, f"step {index}: invalid rotate_degrees params"
        return (
            {
                "playbook": "rotate_degrees",
                "label": label or "Rotate degrees",
                "params": {"degrees": degrees, "speed": speed},
                "deferred_input": False,
            },
            None,
        )

    def _load_sequence_queue_from_json_path(
        self, path: str
    ) -> Tuple[Optional[List[Dict[str, Any]]], List[str]]:
        """
        Load a queue from a JSON file.

        Supported top-level shapes:
        - Fleet request envelope: {"type":"fleet_playbook_request","sequence":[...]}
        - Lightweight local file: {"sequence":[...]} or {"queue":[...]}
        """
        p = os.path.expanduser(path.strip())
        if not p:
            return None, ["Empty file path."]
        if not os.path.isfile(p):
            return None, [f"File not found: {p}"]

        try:
            with open(p, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as ex:
            return None, [f"Failed to parse JSON: {ex}"]

        if not isinstance(data, dict):
            return None, ["Top-level JSON must be an object."]

        raw_steps = data.get("sequence")
        if raw_steps is None:
            raw_steps = data.get("queue")
        if not isinstance(raw_steps, list):
            return None, ["JSON must contain 'sequence' (or 'queue') array."]

        loaded: List[Dict[str, Any]] = []
        errors: List[str] = []
        for idx, raw in enumerate(raw_steps, start=1):
            if not isinstance(raw, dict):
                errors.append(f"step {idx}: must be an object")
                continue
            entry, err = self._normalize_sequence_entry_from_json_step(raw, idx)
            if err:
                errors.append(err)
                continue
            if entry is not None:
                loaded.append(entry)

        if errors:
            return None, errors
        if not loaded:
            return None, ["No valid sequence steps found."]
        return loaded, []

    def _default_presets_dir(self) -> str:
        return os.path.expanduser("~/ros2_ws/src/robot_legion_teleop_python/config/presets")

    def _list_sequence_preset_files(self) -> List[str]:
        base = self._default_presets_dir()
        try:
            names = sorted(
                [
                    n
                    for n in os.listdir(base)
                    if n.lower().endswith(".json")
                    and os.path.isfile(os.path.join(base, n))
                ]
            )
        except Exception:
            return []
        return names

    def _resolve_preset_path(self, user_input: str, default_name: str) -> str:
        """
        Resolve a user-entered preset reference into an absolute JSON file path.

        Rules:
        - empty input -> default_name in presets dir
        - no slash -> treat as preset name under presets dir
        - if name/path has no .json suffix, append it
        """
        raw = str(user_input or "").strip()
        base = self._default_presets_dir()
        if not raw:
            raw = default_name
        # Name-only mode: keep user in presets dir by default.
        if "/" not in raw:
            raw = os.path.join(base, raw)
        path = os.path.expanduser(raw)
        if not path.lower().endswith(".json"):
            path = path + ".json"
        return path

    def _save_sequence_queue_to_json_path(
        self, queue: List[Dict[str, Any]], path: str
    ) -> Tuple[bool, str]:
        if not queue:
            return False, "Queue is empty."
        p = os.path.expanduser(str(path or "").strip())
        if not p:
            return False, "Empty file path."
        parent = os.path.dirname(p) or "."
        try:
            os.makedirs(parent, exist_ok=True)
        except Exception as ex:
            return False, f"Failed to create directory: {ex}"

        sequence: List[Dict[str, Any]] = []
        for i, entry in enumerate(queue, start=1):
            sequence.append(
                {
                    "step_id": f"step-{i}",
                    "label": str(entry.get("label", f"Step {i}")),
                    "playbook": str(entry.get("playbook", "")),
                    "params": dict(entry.get("params", {}) or {}),
                }
            )

        payload = {
            "schema_version": "1.0",
            "type": "fleet_playbook_request",
            "request_id": f"preset.saved.{int(time.time())}",
            "target": {"mode": "all"},
            "execution": {
                "mode": "live",
                "require_confirm": False,
                "non_interactive": True,
                "stop_on_error": False,
                "reachable_timeout_s": 1.0,
            },
            "sequence": sequence,
            "trace": {
                "translator": "terminal_orchestrator",
                "confidence": 1.0,
                "explanation": "saved from execute playbook sequence menu",
            },
        }

        try:
            with open(p, "w", encoding="utf-8") as f:
                json.dump(payload, f, indent=2, sort_keys=True)
                f.write("\n")
        except Exception as ex:
            return False, f"Failed to write preset: {ex}"
        return True, p

    def _execute_sequence_entry(self, entry: Dict[str, Any], seq_index: int, seq_total: int) -> Dict[str, Any]:
        playbook = str(entry.get("playbook", ""))
        if playbook == "move_xy":
            params = entry.get("params", {}) or {}
            live_detected = sorted(self._robots_map().keys())
            configured_detected = [str(r) for r in list(params.get("detected_robots", []))]
            if configured_detected:
                detected_robots = [r for r in configured_detected if r in live_detected]
                if not detected_robots:
                    detected_robots = live_detected
            else:
                detected_robots = live_detected

            raw_main_robot = str(params.get("main_robot", "")).strip()
            if raw_main_robot in detected_robots:
                main_robot = raw_main_robot
            elif detected_robots:
                # Invalid/stale main robot in JSON -> deterministic auto-pick first live robot.
                main_robot = detected_robots[0]
            else:
                main_robot = raw_main_robot

            heading_cfg = dict(params.get("heading_cfg", {}) or {})
            # Ensure every non-main robot has a default heading_clock entry.
            for robot in detected_robots:
                if robot == main_robot:
                    continue
                heading_cfg.setdefault(robot, {"heading_clock": 12.0})

            run = self._run_move_xy_with_params(
                detected_robots=detected_robots,
                x_m=float(params.get("x_m", 1.0)),
                y_m=float(params.get("y_m", 1.0)),
                speed=float(params.get("speed", self.default_speed_scale)),
                max_detour_m=float(params.get("max_detour_m", 0.8)),
                main_robot=main_robot,
                heading_cfg=heading_cfg,
                require_confirm=False,
                render_result=False,
            )
            return {
                "step": seq_index,
                "playbook": "move_xy",
                "label": "Move to objective",
                "status": "completed" if bool(run.get("ok", False)) else "failed",
                "reason": str(run.get("reason", "")),
                "reports": list(run.get("reports", [])),
            }

        if playbook == "execute_all_commands":
            params = entry.get("params", {}) or {}
            speed = max(0.05, min(1.5, float(params.get("speed", 1.0))))
            # Reuse existing implementation behavior by routing through direct command plan.
            # Keep this as the same primitive sweep and target semantics as menu option 2.
            now = int(time.time())
            reports: List[GoalReport] = []
            command_plan = [
                ("hold", {"duration_s": 0.5, "speed": speed}),
                ("transit", {"direction": "forward", "duration_s": 0.8, "speed": speed}),
                ("rotate", {"direction": "left", "duration_s": 0.5, "speed": speed}),
                ("strafe", {"direction": "right", "duration_s": 0.7, "speed": speed}),
                ("diagonal", {"direction": "fwd_right", "duration_s": 0.7, "speed": speed}),
                ("turn", {"direction": "left", "duration_s": 0.7, "speed": speed}),
                ("transit_xy", {"north_m": 0.25, "east_m": 0.15, "speed": speed}),
            ]
            for idx, (command_id, cparams) in enumerate(command_plan, start=1):
                phase = f"seq{seq_index}_c{idx}_{command_id}"

                def build_goal(robot: str, cid=command_id, p=cparams, ph=phase) -> ExecutePlaybook.Goal:
                    g = ExecutePlaybook.Goal()
                    g.intent_id = f"term_seq_all_{ph}_{robot}_{now}"
                    g.command_id = cid
                    g.vehicle_ids = [robot]
                    if cid == "transit_xy":
                        g.north_m = float(p.get("north_m", 0.0))
                        g.east_m = float(p.get("east_m", 0.0))
                    g.parameters_json = json.dumps(p, separators=(",", ":"))
                    return g

                reports.extend(self._dispatch_goal(build_goal))
            return {
                "step": seq_index,
                "playbook": "execute_all_commands",
                "label": "Execute ALL commands",
                "status": "completed",
                "reports": reports,
            }

        if playbook == "transit_distance":
            params = entry.get("params", {}) or {}
            meters = float(params.get("meters", 1.0))
            speed = max(0.05, min(1.5, float(params.get("speed", self.default_speed_scale))))
            direction = "forward" if meters >= 0.0 else "backward"
            now = int(time.time())

            def build_goal(robot: str) -> ExecutePlaybook.Goal:
                g = ExecutePlaybook.Goal()
                g.intent_id = f"term_seq_transit_{seq_index}_{robot}_{now}"
                g.command_id = "transit"
                g.vehicle_ids = [robot]
                linear_scale, _ = self._duration_scales_for_robot(robot)
                duration_s = (
                    abs(meters) / max(1e-6, self.base_linear_mps * speed)
                ) * linear_scale
                g.parameters_json = json.dumps(
                    {
                        "direction": direction,
                        "duration_s": float(duration_s),
                        "speed": float(speed),
                    },
                    separators=(",", ":"),
                )
                return g

            reports = self._dispatch_goal(build_goal)
            return {
                "step": seq_index,
                "playbook": "transit_distance",
                "label": "Transit distance",
                "status": "completed",
                "reports": reports,
            }

        if playbook == "rotate_degrees":
            params = entry.get("params", {}) or {}
            degrees = float(params.get("degrees", 45.0))
            speed = max(0.05, min(1.5, float(params.get("speed", self.default_speed_scale))))
            direction = "left" if degrees >= 0.0 else "right"
            radians = abs(degrees) * math.pi / 180.0
            now = int(time.time())

            def build_goal(robot: str) -> ExecutePlaybook.Goal:
                g = ExecutePlaybook.Goal()
                g.intent_id = f"term_seq_rotate_{seq_index}_{robot}_{now}"
                g.command_id = "rotate"
                g.vehicle_ids = [robot]
                _, angular_scale = self._duration_scales_for_robot(robot)
                duration_s = (
                    radians / max(1e-6, self.base_angular_rps * speed)
                ) * angular_scale
                g.parameters_json = json.dumps(
                    {
                        "direction": direction,
                        "duration_s": float(duration_s),
                        "speed": float(speed),
                    },
                    separators=(",", ":"),
                )
                return g

            reports = self._dispatch_goal(build_goal)
            return {
                "step": seq_index,
                "playbook": "rotate_degrees",
                "label": "Rotate degrees",
                "status": "completed",
                "reports": reports,
            }
        return {
            "step": seq_index,
            "playbook": playbook,
            "label": str(entry.get("label", playbook)),
            "status": "skipped",
            "reason": "unsupported sequence entry",
            "reports": [],
        }

    def _playbook_sequence_menu(self):
        queue: List[Dict[str, Any]] = []
        while rclpy.ok():
            preset_names = self._list_sequence_preset_files()
            lines = [
                "Build and run playbook queue",
                "",
                "Enter 'back' or '0' to",
                "return to the main menu",
                "",
                "Add playbook:",
                "1) Move to objective",
                "2) Execute ALL commands",
                "3) Transit distance",
                "4) Rotate degrees",
                "",
                f"Preset queue files: {len(preset_names)}",
            ]
            if preset_names:
                for n in preset_names[:12]:
                    lines.append(f"- {n}")
                if len(preset_names) > 12:
                    lines.append(f"... ({len(preset_names)} total)")
            else:
                lines.append("- none found")
            if self._last_saved_preset_name:
                lines.append(f"Last saved: {self._last_saved_preset_name}")
            lines.extend(
                [
                "",
                "l) Load queue JSON",
                "n) Save queue as preset",
                "e) Execute queued",
                "d) Delete last queued",
                "c) Clear queue",
                "b) Back",
                "!) STOP ALL ROBOTS",
                "",
                "Entry management:",
                "u) Update by number",
                "x) Delete by number",
                "",
                ]
            )
            if queue:
                lines.append("")
                lines.append("Queued:")
                for i, item in enumerate(queue, start=1):
                    label = str(item.get("label", "playbook"))
                    lines.append(f"{i}) {label}")
            else:
                lines.append("")
                lines.append("Queue empty")

            self._print_screen("EXECUTE PLAYBOOK SEQUENCE", lines)
            raw_choice = input("Choice: ")
            choice = raw_choice.strip().lower()
            if self._is_stop_cmd(raw_choice):
                self._emergency_stop_all()
                continue

            if choice in ("0", "b", "back", "q", "quit"):
                return
            if choice in ("d", "del", "delete"):
                if queue:
                    queue.pop()
                continue
            if choice in ("l", "load"):
                default_name = "fleet_preset_sequence_patrol"
                path_raw = input(
                    f"Preset name [{default_name}] (? lists presets): "
                ).strip()
                if path_raw.lower() in ("b", "back", "0", "q", "quit"):
                    continue
                if path_raw == "?":
                    presets = self._list_sequence_preset_files()
                    lines_show = [f"Preset dir: {self._default_presets_dir()}", ""]
                    if presets:
                        lines_show.extend([f"- {n}" for n in presets])
                    else:
                        lines_show.append("No preset JSON files found.")
                    self._print_screen("AVAILABLE PRESETS", lines_show)
                    self._pause()
                    continue
                path = self._resolve_preset_path(path_raw, default_name)
                mode_raw = input("Load mode: replace or append [replace]: ").strip().lower()
                if mode_raw in ("b", "back", "0", "q", "quit"):
                    continue
                mode = "append" if mode_raw in ("a", "append") else "replace"
                loaded, errors = self._load_sequence_queue_from_json_path(path)
                if loaded is None:
                    self._print_screen(
                        "LOAD QUEUE FAILED",
                        [f"Path: {os.path.expanduser(path)}", ""] + errors[:8],
                    )
                    self._pause()
                    continue
                if mode == "replace":
                    queue = loaded
                else:
                    queue.extend(loaded)
                self._print_screen(
                    "QUEUE LOADED",
                    [
                        f"Path: {os.path.expanduser(path)}",
                        f"Mode: {mode}",
                        f"Loaded steps: {len(loaded)}",
                        f"Queue size: {len(queue)}",
                    ],
                )
                self._pause()
                continue
            if choice in ("n", "new", "save"):
                if not queue:
                    self._print_screen("SAVE PRESET", ["Queue is empty."])
                    self._pause()
                    continue
                default_name = "fleet_preset_custom"
                path_raw = input(f"Save preset name [{default_name}]: ").strip()
                if path_raw.lower() in ("b", "back", "0", "q", "quit"):
                    continue
                path = self._resolve_preset_path(path_raw, default_name)
                ok, detail = self._save_sequence_queue_to_json_path(queue, path)
                if not ok:
                    self._print_screen("SAVE PRESET FAILED", [detail])
                    self._pause()
                    continue
                self._print_screen(
                    "PRESET SAVED",
                    [
                        f"Path: {detail}",
                        f"Name: {os.path.basename(detail)}",
                        f"Queued steps: {len(queue)}",
                        "Use 'l' to load it later.",
                    ],
                )
                self._last_saved_preset_name = os.path.basename(detail)
                self._pause()
                continue
            if choice in ("u", "update"):
                if not queue:
                    continue
                idx_raw = input("Entry number to update: ").strip()
                if idx_raw.lower() in ("b", "back", "0", "q", "quit"):
                    continue
                try:
                    idx = int(idx_raw)
                except ValueError:
                    continue
                if not (1 <= idx <= len(queue)):
                    continue
                current = queue[idx - 1]
                mapped_choice = self._sequence_choice_for_entry(current)
                if mapped_choice is None:
                    continue
                updated = self._collect_sequence_entry(mapped_choice, existing_entry=current)
                if updated is not None:
                    queue[idx - 1] = updated
                continue
            if choice in ("x", "remove", "rm", "delete_num"):
                if not queue:
                    continue
                idx_raw = input("Entry number to delete: ").strip()
                if idx_raw.lower() in ("b", "back", "0", "q", "quit"):
                    continue
                try:
                    idx = int(idx_raw)
                except ValueError:
                    continue
                if 1 <= idx <= len(queue):
                    queue.pop(idx - 1)
                continue
            if choice in ("c", "clear"):
                queue.clear()
                continue
            if choice in ("e", "exec", "execute"):
                if not queue:
                    self._print_screen("EXECUTE PLAYBOOK SEQUENCE", ["Queue is empty."])
                    self._pause()
                    continue
                if not self._confirm(
                    [
                        f"Queued playbooks: {len(queue)}",
                        "Execute in listed order?",
                    ]
                ):
                    continue
                # Snapshot queue for this run.
                to_run = list(queue)
                step_results: List[Dict[str, Any]] = []
                for idx, entry in enumerate(to_run, start=1):
                    step_results.append(
                        self._execute_sequence_entry(entry, seq_index=idx, seq_total=len(to_run))
                    )

                total_reports = 0
                ok_reports = 0
                fail_reports = 0
                skipped_steps = 0
                lines = [
                    f"Steps executed: {len(step_results)}",
                    "",
                ]
                for sr in step_results:
                    label = str(sr.get("label", sr.get("playbook", "playbook")))
                    status = str(sr.get("status", "unknown"))
                    if status == "skipped":
                        skipped_steps += 1
                        lines.append(f"{sr.get('step', '?')}) {label}: SKIP")
                        reason = str(sr.get("reason", "")).strip()
                        if reason:
                            lines.append(f"  {reason}")
                        continue
                    reports = list(sr.get("reports", []))
                    total_reports += len(reports)
                    oks = sum(1 for r in reports if r.accepted and r.success)
                    fails = len(reports) - oks
                    ok_reports += oks
                    fail_reports += fails
                    lines.append(
                        f"{sr.get('step', '?')}) {label}: "
                        f"OK={oks} FAIL={fails}"
                    )

                lines.extend(
                    [
                        "",
                        f"Robot reports: {total_reports}",
                        f"Total OK: {ok_reports}",
                        f"Total FAIL: {fail_reports}",
                        f"Skipped steps: {skipped_steps}",
                    ]
                )
                self._print_screen("SEQUENCE SUMMARY", lines)

                sequence_json = {
                    "type": "playbook_sequence_result",
                    "queued_steps": len(step_results),
                    "robot_reports_total": total_reports,
                    "ok_total": ok_reports,
                    "fail_total": fail_reports,
                    "skipped_steps": skipped_steps,
                    "steps": [
                        {
                            "step": sr.get("step"),
                            "playbook": sr.get("playbook"),
                            "label": sr.get("label"),
                            "status": sr.get("status"),
                            "reason": sr.get("reason", ""),
                            "reports": [
                                {
                                    "robot": r.robot,
                                    "accepted": r.accepted,
                                    "success": r.success,
                                    "reason": r.reason,
                                }
                                for r in list(sr.get("reports", []))
                            ],
                        }
                        for sr in step_results
                    ],
                }
                print(json.dumps(sequence_json, indent=2, sort_keys=True))
                sys.stdout.flush()
                self._pause()
                continue

            if choice in ("1", "2", "3", "4"):
                entry = self._collect_sequence_entry(choice)
                if entry is not None:
                    queue.append(entry)
                continue

    # ---------------- Playbooks ----------------

    def _run_move_xy_with_params(
        self,
        detected_robots: List[str],
        x_m: float,
        y_m: float,
        speed: float,
        max_detour_m: float,
        main_robot: str,
        heading_cfg: Dict[str, Dict[str, float]],
        require_confirm: bool = True,
        render_result: bool = True,
    ) -> Dict[str, Any]:
        """
        Shared executor for playbook 1.

        Used by:
        - interactive playbook 1 screen
        - non-interactive sequence mode
        """
        detected_robots = [str(r) for r in detected_robots]
        if not detected_robots:
            return {"ok": False, "reason": "no detected robots", "reports": []}
        if main_robot not in detected_robots:
            return {"ok": False, "reason": f"main robot '{main_robot}' not in detected robots", "reports": []}

        # Build per-robot target vectors to the SAME objective.
        # Main-frame objective is (x_m, y_m) for all robots.
        goal_vectors: Dict[str, Tuple[float, float]] = {main_robot: (float(x_m), float(y_m))}
        robot_heading_offset_rad: Dict[str, float] = {main_robot: 0.0}
        for robot in detected_robots:
            if robot == main_robot:
                continue
            cfg = heading_cfg.get(robot, {"heading_clock": 12.0})
            heading_clock = float(cfg.get("heading_clock", 12.0))
            heading_rad = self._clock_to_rad(heading_clock)
            robot_heading_offset_rad[robot] = heading_rad
            # Same world/main-frame objective for each robot, transformed into the
            # robot's local frame using its heading offset vs main robot.
            tx_main = float(x_m)
            ty_main = float(y_m)
            tx_robot, ty_robot = self._rotate_vector_into_robot_frame(
                tx_main, ty_main, heading_rad
            )
            goal_vectors[robot] = (tx_robot, ty_robot)

        # Reference straight path is main robot's start->goal line.
        main_goal = goal_vectors.get(main_robot, (float(x_m), float(y_m)))
        ref_line = main_goal if math.hypot(main_goal[0], main_goal[1]) > 1e-9 else (1.0, 0.0)

        # Side preference for non-main robots:
        # 1) Prefer side from dy sign so forward+forward constraints stay feasible.
        #    +y (rightward destination) prefers left-turn geometry,
        #    -y (leftward destination) prefers right-turn geometry.
        # 2) If dy is near zero, use alternating order to maintain separation.
        # 3) If preferred side fails, automatically retry opposite side.
        side_for_robot: Dict[str, str] = {}
        non_main_order = [r for r in detected_robots if r != main_robot]
        alt_side_hint: Dict[str, str] = {}
        for i, robot in enumerate(non_main_order):
            alt_side_hint[robot] = "left" if (i % 2 == 0) else "right"

        # Main objective heading (used for main robot shortest-path plan).
        main_heading = (
            math.atan2(float(main_goal[1]), float(main_goal[0]))
            if (abs(main_goal[0]) + abs(main_goal[1])) > 1e-9
            else 0.0
        )

        # Detour assignment rule:
        # - main robot: 0 (straight path)
        # - "second robot": next detected robot (excluding main) gets max_detour_m
        # - remaining robots: evenly spaced detour levels between 0 and max_detour_m
        detour_targets: Dict[str, float] = {main_robot: 0.0}
        non_main = [r for r in detected_robots if r != main_robot]
        secondary_robot = non_main[0] if non_main else None
        if secondary_robot:
            detour_targets[secondary_robot] = max_detour_m
        remaining = [r for r in non_main if r != secondary_robot]
        if remaining:
            step = max_detour_m / float(len(remaining) + 1)
            for i, robot in enumerate(remaining, start=1):
                detour_targets[robot] = i * step

        plans: Dict[str, TwoLegPlan] = {}
        fallback_mode: Dict[str, str] = {}
        failures: List[str] = []
        for robot in detected_robots:
            dx, dy = goal_vectors[robot]
            if robot == main_robot:
                # Main robot path: one rotate + one transit (straight to objective).
                heading = math.atan2(dy, dx) if (abs(dx) + abs(dy)) > 1e-9 else 0.0
                dist = math.hypot(dx, dy)
                plans[robot] = TwoLegPlan(
                    theta1=heading,
                    theta2=heading,
                    length1_m=dist,
                    length2_m=0.0,
                    max_detour_m=abs(self._signed_distance_to_line((dx, dy), ref_line)),
                )
                fallback_mode[robot] = "straight_main"
            else:
                if dy > 1e-6:
                    preferred_side = "left"
                elif dy < -1e-6:
                    preferred_side = "right"
                else:
                    preferred_side = alt_side_hint.get(robot, "left")
                fallback_side = "right" if preferred_side == "left" else "left"
                candidate_sides = [preferred_side, fallback_side]

                best_plan: Optional[TwoLegPlan] = None
                best_side: Optional[str] = None
                best_score: Optional[Tuple[float, float]] = None
                detour_scale = self._detour_scale_for_robot(robot)
                target_detour = float(detour_targets.get(robot, 0.0)) * detour_scale
                max_detour_for_robot = float(max_detour_m) * detour_scale
                for side in candidate_sides:
                    candidate = self._plan_two_leg_vector_sum(
                        dx=dx,
                        dy=dy,
                        turn_side=side,
                        target_detour_m=target_detour,
                        max_detour_m=max_detour_for_robot,
                        ref_line_dir_xy=ref_line,
                    )
                    score = (
                        abs(candidate.max_detour_m - target_detour),
                        candidate.length1_m + candidate.length2_m,
                    )
                    if best_score is None or score < best_score:
                        best_plan = candidate
                        best_side = side
                        best_score = score

                plan = best_plan
                if plan is None:
                    failures.append(robot)
                    continue
                side_for_robot[robot] = best_side or preferred_side
                fallback_mode[robot] = "v_plan"
                plans[robot] = plan

        if failures:
            return {
                "ok": False,
                "reason": (
                    "No feasible deterministic plan under current constraints for: "
                    + ", ".join(failures)
                ),
                "reports": [],
            }

        est_total_seq = 0.0
        for robot in detected_robots:
            p = plans[robot]
            r1 = abs(p.theta1) / max(1e-6, self.base_angular_rps * speed)
            t1 = abs(p.length1_m) / max(1e-6, self.base_linear_mps * speed)
            r2 = abs(p.theta2 - p.theta1) / max(1e-6, self.base_angular_rps * speed)
            t2 = abs(p.length2_m) / max(1e-6, self.base_linear_mps * speed)
            est_total_seq = max(est_total_seq, r1 + t1 + r2 + t2)

        target_text = f"detected={len(detected_robots)}"
        summary = [
            f"Target: {target_text}",
            "Mode: detected-robot XY",
            "Intent: same objective all robots",
            "",
            f"Main robot: {main_robot}",
            f"Base x={x_m:.2f} y={y_m:.2f}",
            "Main path: shortest straight",
            "Non-main: V path (2 legs)",
            "V-plan: two vectors sum",
            "to destination vector",
            "Side assignment: auto",
            "(dy-sign preferred, fallback)",
            f"Main heading={math.degrees(main_heading):.1f} deg",
            f"MAX PATH OFFSET={max_detour_m:.2f} m",
            "",
            f"Est phase total: {est_total_seq:.2f} s",
            "",
            "Frame: robot-relative",
            "(heading offsets applied)",
        ]

        for robot in detected_robots:
            dx, dy = goal_vectors[robot]
            p = plans[robot]
            summary.append(f"{robot}: dx={dx:.2f} dy={dy:.2f}")
            summary.append(
                f"  th1={math.degrees(p.theta1):.1f}deg "
                f"th2={math.degrees(p.theta2):.1f}deg "
                f"detour={p.max_detour_m:.2f}m "
                f"target={detour_targets.get(robot, 0.0):.2f}m"
            )
            summary.append(f"  mode={fallback_mode.get(robot, 'v_plan')}")
            if robot != main_robot:
                if fallback_mode.get(robot) == "v_plan":
                    summary.append(f"  side={side_for_robot.get(robot, 'left')}")
                cfg = heading_cfg.get(robot, {"heading_clock": 12.0})
                summary.append(
                    f"  heading={float(cfg.get('heading_clock', 12.0)):.1f} o'clock"
                )

        if require_confirm and (not self._confirm(summary)):
            return {"ok": False, "reason": "user cancelled", "reports": []}

        now = int(time.time())
        phase_reports: List[GoalReport] = []

        # Build phase helpers.
        def _build_phase_goal(robot: str, phase_label: str, command_id: str, params: Dict[str, object]) -> ExecutePlaybook.Goal:
            g = ExecutePlaybook.Goal()
            g.intent_id = f"term_xy_{phase_label}_{robot}_{now}"
            g.command_id = command_id
            g.vehicle_ids = [robot]
            g.parameters_json = json.dumps(params, separators=(",", ":"))
            return g

        # Emit a high-level planner event for future fleet_orchestrator integration.
        self._emit_json_event(
            "task",
            {
                "type": "playbook_command",
                "intent_id": f"term_xy_plan_{now}",
                "command_id": "transit_xy_deterministic_formation",
                "platform": {
                    "adapter_type": "terminal_orchestrator",
                    "platform_family": "robot_legion_teleop_python",
                },
                "targets": detected_robots,
                "parameters": {
                    "x_m": x_m,
                    "y_m": y_m,
                    "speed": speed,
                    "heading_config": heading_cfg,
                    "side_assignment_mode": "auto_dy_sign_with_fallback",
                    "final_heading_mode": "non_main_v_path_variable_heading",
                    "main_heading_deg": math.degrees(main_heading),
                    "max_path_offset_m": max_detour_m,
                    "main_robot": main_robot,
                    "secondary_robot": secondary_robot,
                },
                "trace": {
                    "translator": "terminal_orchestrator",
                    "confidence": 1.0,
                    "explanation": "deterministic two-leg linear algebra planner",
                },
                "plans": {
                    robot: {
                        "dx": goal_vectors[robot][0],
                        "dy": goal_vectors[robot][1],
                        "heading_offset_deg": math.degrees(
                            robot_heading_offset_rad.get(robot, 0.0)
                        ),
                        "theta1_deg": math.degrees(plans[robot].theta1),
                        "theta2_deg": math.degrees(plans[robot].theta2),
                        "length1_m": plans[robot].length1_m,
                        "length2_m": plans[robot].length2_m,
                        "planner_mode": fallback_mode.get(robot, "v_plan"),
                        "max_detour_m": plans[robot].max_detour_m,
                        "target_detour_m": detour_targets.get(robot, 0.0),
                    }
                    for robot in detected_robots
                },
            },
        )

        # Phase 1: first rotate for all robots (parallel dispatch).
        def build_phase1(robot: str) -> ExecutePlaybook.Goal:
            p = plans[robot]
            params = self._rotation_params_from_delta(
                delta_rad=p.theta1, speed_scale=speed, robot=robot
            )
            return _build_phase_goal(robot, "p1_rotate", "rotate", params)

        phase1_reports = self._dispatch_goal(build_phase1, target_robots=detected_robots)
        phase_reports.extend(phase1_reports)
        if len(phase1_reports) == 0:
            reason = (
                "Move-to-objective failed: phase 1 rotate had zero dispatched targets. "
                "No reachable robots responded."
            )
            if render_result:
                self._print_screen("MOVE TO OBJECTIVE FAILED", [reason])
                self._pause()
            return {"ok": False, "reason": reason, "reports": phase_reports}
        if any((not r.success) for r in phase_reports[-len(phase1_reports):]):
            if render_result:
                self._render_reports(phase_reports)
            return {"ok": False, "reason": "phase 1 failed", "reports": phase_reports}

        # Phase 2: first transit for all robots.
        def build_phase2(robot: str) -> ExecutePlaybook.Goal:
            p = plans[robot]
            params = self._transit_params_from_length(
                length_m=p.length1_m, speed_scale=speed, robot=robot
            )
            return _build_phase_goal(robot, "p2_transit", "transit", params)

        phase2_reports = self._dispatch_goal(build_phase2, target_robots=detected_robots)
        phase_reports.extend(phase2_reports)
        if len(phase2_reports) == 0:
            reason = (
                "Move-to-objective failed: phase 2 transit had zero dispatched targets. "
                "No reachable robots responded."
            )
            if render_result:
                self._print_screen("MOVE TO OBJECTIVE FAILED", [reason])
                self._pause()
            return {"ok": False, "reason": reason, "reports": phase_reports}
        if any((not r.success) for r in phase_reports[-len(phase2_reports):]):
            if render_result:
                self._render_reports(phase_reports)
            return {"ok": False, "reason": "phase 2 failed", "reports": phase_reports}

        # Phase 3: second rotate for all robots.
        def build_phase3(robot: str) -> ExecutePlaybook.Goal:
            p = plans[robot]
            params = self._rotation_params_from_delta(
                delta_rad=(p.theta2 - p.theta1), speed_scale=speed, robot=robot
            )
            return _build_phase_goal(robot, "p3_rotate", "rotate", params)

        phase3_reports = self._dispatch_goal(build_phase3, target_robots=detected_robots)
        phase_reports.extend(phase3_reports)
        if len(phase3_reports) == 0:
            reason = (
                "Move-to-objective failed: phase 3 rotate had zero dispatched targets. "
                "No reachable robots responded."
            )
            if render_result:
                self._print_screen("MOVE TO OBJECTIVE FAILED", [reason])
                self._pause()
            return {"ok": False, "reason": reason, "reports": phase_reports}
        if any((not r.success) for r in phase_reports[-len(phase3_reports):]):
            if render_result:
                self._render_reports(phase_reports)
            return {"ok": False, "reason": "phase 3 failed", "reports": phase_reports}

        # Phase 4: second transit for all robots.
        def build_phase4(robot: str) -> ExecutePlaybook.Goal:
            p = plans[robot]
            params = self._transit_params_from_length(
                length_m=p.length2_m, speed_scale=speed, robot=robot
            )
            return _build_phase_goal(robot, "p4_transit", "transit", params)

        phase4_reports = self._dispatch_goal(build_phase4, target_robots=detected_robots)
        phase_reports.extend(phase4_reports)
        if len(phase4_reports) == 0:
            reason = (
                "Move-to-objective failed: phase 4 transit had zero dispatched targets. "
                "No reachable robots responded."
            )
            if render_result:
                self._print_screen("MOVE TO OBJECTIVE FAILED", [reason])
                self._pause()
            return {"ok": False, "reason": reason, "reports": phase_reports}
        if render_result:
            self._render_reports(phase_reports)
        return {"ok": True, "reason": "ok", "reports": phase_reports}

    def _playbook_move_xy(self):
        self._print_screen(
            "PLAYBOOK: MOVE XY",
            [
                "detected-robot objective",
                "x: forward(+) backward(-)",
                "y: right(+) left(-)",
                "main robot goes straight",
                "others use V-path planner",
                "",
                "Type 'back' to cancel",
            ],
        )
        # Requirement: this playbook targets ALL currently detected robots.
        detected_robots = sorted(self._robots_map().keys())
        if not detected_robots:
            self._print_screen(
                "MOVE XY REQUIREMENT",
                [
                    "No detected robots.",
                    "Start robot bringup and retry.",
                ],
            )
            self._pause()
            return

        x_m = self._prompt_float("x meters", default=1.0)
        if x_m is None:
            return
        y_m = self._prompt_float("y meters", default=1.0)
        if y_m is None:
            return
        speed = self._prompt_float("speed scale", default=self.default_speed_scale, allow_zero=False)
        if speed is None:
            return
        speed = max(0.05, min(1.5, speed))
        max_detour_m = self._prompt_float("MAX PATH OFFSET m", default=0.8, allow_zero=False)
        if max_detour_m is None:
            return
        max_detour_m = max(0.05, float(max_detour_m))

        self._print_screen(
            "MAIN ROBOT SELECT",
            [
                "Choose main robot (anchor)",
                "Formation offsets are",
                "interpreted from this robot.",
                "",
            ] + [f"{i+1}) {r}" for i, r in enumerate(detected_robots)],
        )
        while True:
            main_choice = input("Main robot index [1]: ").strip()
            if main_choice == "":
                main_idx = 1
                break
            if main_choice.lower() in ("b", "back", "q", "quit"):
                return
            try:
                parsed = int(main_choice)
            except ValueError:
                print("Invalid index. Auto-selecting 1.")
                main_idx = 1
                break
            if 1 <= parsed <= len(detected_robots):
                main_idx = parsed
                break
            print("Invalid index. Auto-selecting 1.")
            main_idx = 1
            break
        main_robot = detected_robots[main_idx - 1]

        default_head = self._default_heading_config(
            detected_robots=detected_robots,
            main_robot=main_robot,
        )
        heading_cfg = self._prompt_heading_config(
            detected_robots=detected_robots,
            main_robot=main_robot,
            defaults=default_head,
        )
        if heading_cfg is None:
            return

        run = self._run_move_xy_with_params(
            detected_robots=detected_robots,
            x_m=float(x_m),
            y_m=float(y_m),
            speed=float(speed),
            max_detour_m=float(max_detour_m),
            main_robot=main_robot,
            heading_cfg=heading_cfg,
            require_confirm=True,
            render_result=True,
        )
        if not bool(run.get("ok", False)):
            reason = str(run.get("reason", "")).strip()
            # Keep user-cancel as quiet back-to-menu behavior.
            if reason and reason != "user cancelled":
                self._print_screen(
                    "MOVE TO OBJECTIVE FAILED",
                    [reason],
                )
                self._pause()

    def _playbook_execute_all_commands(self):
        """
        Validation sweep:
        Execute one safe example of each supported playbook primitive on all target robots.
        This is intended as a quick integration test, not mission behavior.
        """
        self._print_screen(
            "PLAYBOOK: ALL COMMANDS",
            [
                "Runs all primitive commands",
                "on selected target robots.",
                "",
                "Type 'back' to cancel",
            ],
        )

        speed = self._prompt_float("speed scale", default=1.0, allow_zero=False)
        if speed is None:
            return
        speed = max(0.05, min(1.5, speed))

        target_text = self.selected_robot or "ALL"
        summary = [
            f"Target: {target_text}",
            "",
            "Command sweep order:",
            "1) hold",
            "2) transit",
            "3) rotate",
            "4) strafe",
            "5) diagonal",
            "6) turn",
            "7) transit_xy",
            "",
            f"Speed scale: {speed:.2f}",
            "Purpose: playbook smoke test",
        ]
        if not self._confirm(summary):
            return

        now = int(time.time())
        reports: List[GoalReport] = []

        # Fixed safe-ish parameter set for deterministic testing.
        command_plan = [
            ("hold", {"duration_s": 0.5, "speed": speed}),
            ("transit", {"direction": "forward", "duration_s": 0.8, "speed": speed}),
            ("rotate", {"direction": "left", "duration_s": 0.5, "speed": speed}),
            ("strafe", {"direction": "right", "duration_s": 0.7, "speed": speed}),
            ("diagonal", {"direction": "fwd_right", "duration_s": 0.7, "speed": speed}),
            ("turn", {"direction": "left", "duration_s": 0.7, "speed": speed}),
            ("transit_xy", {"north_m": 0.25, "east_m": 0.15, "speed": speed}),
        ]

        self._emit_json_event(
            "task",
            {
                "type": "playbook_command",
                "intent_id": f"term_all_commands_plan_{now}",
                "command_id": "playbook_command_sweep",
                "platform": {
                    "adapter_type": "terminal_orchestrator",
                    "platform_family": "robot_legion_teleop_python",
                },
                "targets": target_text,
                "parameters": {"speed": speed, "command_count": len(command_plan)},
                "trace": {
                    "translator": "terminal_orchestrator",
                    "confidence": 1.0,
                    "explanation": "manual all-commands validation sweep",
                },
            },
        )

        for idx, (command_id, params) in enumerate(command_plan, start=1):
            phase = f"c{idx}_{command_id}"

            def build_goal(robot: str, cid=command_id, p=params, ph=phase) -> ExecutePlaybook.Goal:
                g = ExecutePlaybook.Goal()
                g.intent_id = f"term_all_{ph}_{robot}_{now}"
                g.command_id = cid
                g.vehicle_ids = [robot]
                if cid == "transit_xy":
                    g.north_m = float(p.get("north_m", 0.0))
                    g.east_m = float(p.get("east_m", 0.0))
                g.parameters_json = json.dumps(p, separators=(",", ":"))
                return g

            # Dispatch each primitive in synchronized phase order across robots.
            reports.extend(self._dispatch_goal(build_goal))

        self._render_reports(reports)

    def _playbook_transit_distance(self):
        self._print_screen(
            "PLAYBOOK: TRANSIT DIST",
            [
                "Move forward/back by meters",
                "distance + => forward",
                "distance - => backward",
                "",
                "Type 'back' to cancel",
            ],
        )
        meters = self._prompt_float("distance meters", default=1.0, allow_zero=False)
        if meters is None:
            return
        speed = self._prompt_float("speed scale", default=self.default_speed_scale, allow_zero=False)
        if speed is None:
            return
        speed = max(0.05, min(1.5, speed))

        direction = "forward" if meters >= 0.0 else "backward"
        target_text = self.selected_robot or "ALL"
        if self.selected_robot:
            linear_scale, _ = self._duration_scales_for_robot(self.selected_robot)
            duration_s = (
                abs(meters) / max(1e-6, self.base_linear_mps * speed)
            ) * linear_scale
        else:
            duration_s = abs(meters) / max(1e-6, self.base_linear_mps * speed)
        summary = [
            f"Target: {target_text}",
            "",
            f"Distance: {meters:.2f} m",
            f"Direction: {direction}",
            (
                f"Duration: {duration_s:.2f} s"
                if self.selected_robot
                else f"Base duration: {duration_s:.2f} s"
            ),
            (
                "Per-robot linear calibration "
                "applied at dispatch"
                if not self.selected_robot
                else ""
            ),
        ]
        if not self._confirm(summary):
            return

        now = int(time.time())

        def build_goal(robot: str) -> ExecutePlaybook.Goal:
            g = ExecutePlaybook.Goal()
            g.intent_id = f"term_transit_{robot}_{now}"
            g.command_id = "transit"
            g.vehicle_ids = [robot]
            linear_scale, _ = self._duration_scales_for_robot(robot)
            robot_duration_s = (
                abs(meters) / max(1e-6, self.base_linear_mps * speed)
            ) * linear_scale
            params = {
                "direction": direction,
                "duration_s": float(robot_duration_s),
                "speed": float(speed),
            }
            g.parameters_json = json.dumps(params, separators=(",", ":"))
            return g

        reports = self._dispatch_goal(build_goal)
        self._render_reports(reports)

    def _playbook_rotate_degrees(self):
        self._print_screen(
            "PLAYBOOK: ROTATE DEG",
            [
                "Rotate by degrees",
                "degrees + => left/ccw",
                "degrees - => right/cw",
                "",
                "Type 'back' to cancel",
            ],
        )
        degrees = self._prompt_float("degrees", default=45.0, allow_zero=False)
        if degrees is None:
            return
        speed = self._prompt_float("speed scale", default=self.default_speed_scale, allow_zero=False)
        if speed is None:
            return
        speed = max(0.05, min(1.5, speed))

        direction = "left" if degrees >= 0.0 else "right"
        radians = abs(degrees) * 3.141592653589793 / 180.0
        target_text = self.selected_robot or "ALL"
        if self.selected_robot:
            _, angular_scale = self._duration_scales_for_robot(self.selected_robot)
            duration_s = (
                radians / max(1e-6, self.base_angular_rps * speed)
            ) * angular_scale
        else:
            duration_s = radians / max(1e-6, self.base_angular_rps * speed)
        summary = [
            f"Target: {target_text}",
            "",
            f"Rotate: {degrees:.2f} deg",
            f"Direction: {direction}",
            (
                f"Duration: {duration_s:.2f} s"
                if self.selected_robot
                else f"Base duration: {duration_s:.2f} s"
            ),
            (
                "Per-robot angular calibration "
                "applied at dispatch"
                if not self.selected_robot
                else ""
            ),
        ]
        if not self._confirm(summary):
            return

        now = int(time.time())

        def build_goal(robot: str) -> ExecutePlaybook.Goal:
            g = ExecutePlaybook.Goal()
            g.intent_id = f"term_rotate_{robot}_{now}"
            g.command_id = "rotate"
            g.vehicle_ids = [robot]
            _, angular_scale = self._duration_scales_for_robot(robot)
            robot_duration_s = (
                radians / max(1e-6, self.base_angular_rps * speed)
            ) * angular_scale
            params = {
                "direction": direction,
                "duration_s": float(robot_duration_s),
                "speed": float(speed),
            }
            g.parameters_json = json.dumps(params, separators=(",", ":"))
            return g

        reports = self._dispatch_goal(build_goal)
        self._render_reports(reports)

    # ---------------- Main loop ----------------

    def _main_menu(self) -> str:
        # Keep graph data fresh before rendering choices.
        rclpy.spin_once(self, timeout_sec=0.15)
        robots = sorted(self._robots_map().keys())
        active_target = self.selected_robot if self.selected_robot else "ALL"
        override_robots = [r for r in robots if self._is_human_override_active(r)]
        lines = [
            "Terminal playbook UI",
            "",
            f"Detected robots: {len(robots)}",
            "Human override: " + (", ".join(override_robots) if override_robots else "none"),
            f"Target: {active_target}",
            "",
            "1) Move to objective",
            "2) Execute ALL commands",
            "3) Transit distance",
            "4) Rotate degrees",
            "",
            "s) Playbook sequence",
            "",
            "!) STOP ALL ROBOTS",
            "",
            "t) Select target",
            "r) Refresh",
            "q) Quit",
        ]
        if robots:
            lines.append("")
            lines.append("Robots:")
            for robot in robots:
                lines.append(f"- {robot}")
        else:
            lines.append("")
            lines.append("No playbook endpoints")
        self._print_screen("TERMINAL ORCHESTRATOR", lines)
        raw_choice = input("Choice: ")
        if self._is_stop_cmd(raw_choice):
            return "__STOP__"
        return raw_choice.strip().lower()

    def run_ui(self):
        self._warmup_discovery(timeout_sec=1.2, spin_step_sec=0.1)
        while rclpy.ok():
            choice = self._main_menu()
            if choice == "__STOP__":
                self._emergency_stop_all()
                continue
            if choice == "q":
                return
            if choice == "r":
                # Redraw loop after a short graph update spin.
                rclpy.spin_once(self, timeout_sec=0.2)
                continue
            if choice == "t":
                self._target_menu()
                continue
            if choice == "s":
                self._playbook_sequence_menu()
                continue
            if choice == "1":
                self._playbook_move_xy()
                continue
            if choice == "2":
                self._playbook_execute_all_commands()
                continue
            if choice == "3":
                self._playbook_transit_distance()
                continue
            if choice == "4":
                self._playbook_rotate_degrees()
                continue


def main(args=None):
    rclpy.init(args=args)
    node = TerminalOrchestrator()
    try:
        node.run_ui()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
