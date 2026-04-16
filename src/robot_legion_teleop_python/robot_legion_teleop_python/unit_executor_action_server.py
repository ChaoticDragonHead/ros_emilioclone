#!/usr/bin/env python3
# SPDX-License-Identifier: LicenseRef-Proprietary

"""
unit_executor_action_server.py

ROLE
----
Robot-side ActionServer:
  /<robot_name>/execute_playbook   fleet_orchestrator_interfaces/action/ExecutePlaybook

It executes playbook primitives by publishing Twist to:
  /<robot_name>/cmd_vel

WHY THIS IS THE MOST IMPORTANT "FLEET" NODE
-------------------------------------------
This is the robot-side boundary for your entire DIU-style demo pipeline:

Voice -> STT -> LLM -> ordered playbooks -> fleet_orchestrator -> Action goals -> robots

Teleop is "manual mode".
This executor is "orchestrated mode".
Both converge on cmd_vel.

That means:
- Your drive-type differences are hidden behind motor_driver_node.
- Your fleet-level logic doesn't fork per drive type.
"""

from __future__ import annotations

import getpass
import json
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

from geometry_msgs.msg import Twist
from std_msgs.msg import String
import threading

try:
    from fleet_orchestrator_interfaces.action import ExecutePlaybook
    HAS_FLEET_ACTION = True
except Exception:
    ExecutePlaybook = None  # type: ignore[assignment]
    HAS_FLEET_ACTION = False

from .drive_profiles import load_profile_registry, resolve_robot_profile
from .playbook_helpers import TimedTwistPlan, run_timed_twist
from .playbook_contract import validate_and_normalize
from .playbook_strategies import compile_transit_xy_plans
from .audit_logger import AuditLogger


class UnitExecutor(Node):
    def __init__(self):
        super().__init__("unit_executor")

        # Robot identity follows your convention.
        self.declare_parameter("robot_name", getpass.getuser())
        self.declare_parameter("profiles_path", "")

        # Default speeds (baseline). Orchestrator can scale using speed= in JSON.
        self.declare_parameter("default_transit_speed_mps", 0.40)
        self.declare_parameter("default_strafe_speed_mps", 0.35)
        self.declare_parameter("default_rotate_speed_rps", 2.00)
        self.declare_parameter("default_turn_linear_mps", 0.18)
        self.declare_parameter("has_magnetometer", False)
        self.declare_parameter("heading_rad", 0.0)
        self.declare_parameter("heading_topic", "")
        self.declare_parameter("teleop_override_ttl_s", 0.7)

        self.robot = str(self.get_parameter("robot_name").value).strip() or getpass.getuser()

        profiles_path = str(self.get_parameter("profiles_path").value).strip() or None
        profiles_path = str(self.get_parameter("profiles_path").value).strip() or None

        prof = None
        self.drive_params = {}
        self.hardware_params = {}
        try:
            reg = load_profile_registry(profiles_path)
            prof = resolve_robot_profile(reg, self.robot)
            self.drive_type = prof["drive_type"]
        except Exception:
            # Fallback: accept explicit parameters or defaults when registry doesn't contain this robot.
            self.declare_parameter("drive_type", "diff_drive")
            self.declare_parameter("hardware", "L298N_diff")
            self.drive_type = str(self.get_parameter("drive_type").value).strip() or "diff_drive"
            self.hardware = str(self.get_parameter("hardware").value).strip() or "L298N_diff"
            self.profile_name = None
            self.get_logger().warning(f"[{self.robot}] profile not found in registry; falling back to drive_type={self.drive_type} hardware={self.hardware}")
        else:
            self.hardware = prof.get("hardware")
            self.profile_name = prof.get("profile_name")
            self.drive_params = prof.get("drive_params", {}) or {}
            self.hardware_params = prof.get("hardware_params", {}) or {}

        profile_has_mag = bool(self.hardware_params.get("has_magnetometer", False))
        param_has_mag = bool(self.get_parameter("has_magnetometer").value)
        self.has_magnetometer = bool(profile_has_mag or param_has_mag)
        self.cardinal_mode = "global_cardinal" if self.has_magnetometer else "relative_fallback"
        self.heading_topic = str(self.get_parameter("heading_topic").value).strip() or f"/{self.robot}/heading_rad"
        self._latest_heading_rad = float(self.get_parameter("heading_rad").value)

        # FUTURE: live heading ingestion from IMU/magnetometer topic
        # -----------------------------------------------------------------
        # Keep this disabled until a heading publisher exists on the robot.
        # This lets fleet_orchestrator keep a stable north/east contract while
        # sensor-equipped robots upgrade to true cardinal behavior robot-side.
        #
        # Example expected topic:
        #   /<robot>/heading_rad   (std_msgs/msg/Float32)
        #
        # To enable later:
        # 1) Uncomment import near top:
        #      from std_msgs.msg import Float32
        # 2) Uncomment the subscription below.
        # 3) Ensure your magnetometer/IMU node publishes heading_rad where:
        #      0.0 rad means robot facing north
        #      +pi/2 rad means robot facing east
        #
        # if self.has_magnetometer:
        #     self._heading_sub = self.create_subscription(
        #         Float32,
        #         self.heading_topic,
        #         self._on_heading_rad,
        #         10,
        #     )

        # Load baseline speeds
        self.v = float(self.get_parameter("default_transit_speed_mps").value)
        self.vy = float(self.get_parameter("default_strafe_speed_mps").value)
        self.w = float(self.get_parameter("default_rotate_speed_rps").value)
        self.turn_v = float(self.get_parameter("default_turn_linear_mps").value)

        self.cmd_vel_topic = f"/{self.robot}/cmd_vel"
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.human_override_topic = f"/{self.robot}/human_override"
        self.teleop_override_ttl_s = float(self.get_parameter("teleop_override_ttl_s").value)
        self._human_override_event = threading.Event()
        self._last_human_override_monotonic = 0.0
        self.human_override_sub = self.create_subscription(
            String,
            self.human_override_topic,
            self._human_override_cb,
            20,
        )

        self.action_name = f"/{self.robot}/execute_playbook"
        self.command_topic = f"/{self.robot}/execute_playbook_cmd"
        self.result_topic = f"/{self.robot}/execute_playbook_result"
        self.result_pub = self.create_publisher(String, self.result_topic, 10)
        self.command_sub = self.create_subscription(
            String,
            self.command_topic,
            self._command_topic_cb,
            10,
        )

        self.server = None
        if HAS_FLEET_ACTION:
            self.server = ActionServer(
                self,
                ExecutePlaybook,
                self.action_name,
                execute_callback=self.execute_cb,
                goal_callback=self.goal_cb,
                cancel_callback=self.cancel_cb,
            )
        else:
            self.get_logger().warning(
                f"[{self.robot}] fleet_orchestrator_interfaces missing; "
                f"running standalone topic interface on {self.command_topic}"
            )

        # Active goals mapping: id(goal_handle) -> threading.Event
        self._active_goals = {}

        # Audit logging for DIU compliance
        audit_log_path = os.environ.get("ROBOT_AUDIT_LOG_PATH") or f"/tmp/robot_{self.robot}_audit.jsonl"
        self.audit = AuditLogger(self, "unit_executor", audit_log_path)

        self.get_logger().info(f"[{self.robot}] UnitExecutor ready")
        if HAS_FLEET_ACTION:
            self.get_logger().info(f"[{self.robot}] action={self.action_name}")
        self.get_logger().info(f"[{self.robot}] cmd_iface={self.command_topic}")
        self.get_logger().info(
            f"[{self.robot}] drive_type={self.drive_type} hardware={self.hardware} "
            f"profile={self.profile_name} cmd_vel={self.cmd_vel_topic}"
        )
        self.get_logger().info(
            f"[{self.robot}] cardinal_mode={self.cardinal_mode} "
            f"(north/east API {'uses heading conversion' if self.has_magnetometer else 'maps to forward/right fallback'})"
        )
        self.get_logger().info(
            f"[{self.robot}] heading source="
            f"{self.heading_topic} (live subscription disabled; using heading_rad param/cache)"
        )
        self.get_logger().info(f"[{self.robot}] Audit log: {audit_log_path}")

    def _human_override_cb(self, msg: String):
        """
        Teleop priority channel.
        Expected values:
        - plain: "1"/"true"/"active" to assert human control
        - JSON: {"active": true}
        """
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
            self._last_human_override_monotonic = time.monotonic()
            self._human_override_event.set()
            # Preempt active autonomous commands immediately.
            for ev in list(self._active_goals.values()):
                try:
                    ev.set()
                except Exception:
                    pass
            # Immediate brake command.
            try:
                self._publish(Twist())
            except Exception:
                pass
        else:
            self._human_override_event.clear()

    def _is_human_override_active(self) -> bool:
        if not self._human_override_event.is_set():
            return False
        age = time.monotonic() - float(self._last_human_override_monotonic)
        if age > max(0.05, float(self.teleop_override_ttl_s)):
            self._human_override_event.clear()
            return False
        return True

    class _CompositeStopEvent:
        def __init__(self, a, b):
            self._a = a
            self._b = b

        def is_set(self):
            a_set = bool(self._a.is_set()) if self._a is not None else False
            b_set = bool(self._b.is_set()) if self._b is not None else False
            return a_set or b_set

    # FUTURE: live heading callback from magnetometer/IMU topic.
    # def _on_heading_rad(self, msg):
    #     """
    #     Expected msg type: std_msgs/msg/Float32
    #     Value convention:
    #       0.0 -> facing north
    #       +pi/2 -> facing east
    #     """
    #     try:
    #         self._latest_heading_rad = float(msg.data)
    #     except Exception:
    #         pass

    # ---------------- action callbacks ----------------
    def goal_cb(self, goal_request):
        """
        Validate early (fast reject) so orchestrator gets immediate feedback.
        """
        ok, err, _parsed = validate_and_normalize(goal_request.command_id, goal_request.parameters_json)
        intent_id = goal_request.intent_id if hasattr(goal_request, "intent_id") else "unknown"

        if not ok:
            self.get_logger().warn(f"[{self.robot}] reject goal: {err}")
            self.audit.log_command(
                robot=self.robot,
                source="orchestrator",
                command_id=goal_request.command_id,
                parameters=json.loads(goal_request.parameters_json or "{}"),
                status="rejected",
                source_id=intent_id,
                details=err,
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        """
        Accept cancels.

        Note: run_timed_twist is a blocking loop. For full cancel correctness,
        we'd refactor run_timed_twist into a timer-driven state machine.
        For Sprint-1 demos, we keep it simple and accept cancel requests.
        """
        # Signal the running executor loop (if any) to stop
        ev = self._active_goals.get(id(goal_handle))
        if ev is not None:
            try:
                ev.set()
            except Exception:
                pass
        return CancelResponse.ACCEPT

    # ---------------- helpers ----------------
    def _publish(self, twist: Twist):
        self.cmd_pub.publish(twist)

    def _run_plan_sequence(self, plans, fb, stop_event):
        """
        Run a sequence of TimedTwistPlan objects while reporting global progress.
        """
        if not plans:
            return
        total = float(len(plans))
        for idx, plan in enumerate(plans):
            if stop_event is not None and stop_event.is_set():
                break

            def _phase_fb(percent, text, phase_idx=idx):
                global_percent = ((phase_idx + (float(percent) / 100.0)) / total) * 100.0
                fb(global_percent, text)

            run_timed_twist(
                self._publish,
                _phase_fb,
                plan,
                rate_hz=20.0,
                stop_at_end=True,
                stop_event=stop_event,
            )

    def _publish_feedback_safe(self, goal_handle, feedback_msg, percent: float, text: str):
        """
        Your earlier logs showed an AttributeError around percent_complete.

        That happens when the action definition differs between builds.
        So we write feedback defensively:
          - if field exists, set it
          - if not, ignore

        This makes the executor resilient across interface revisions.
        """
        if hasattr(feedback_msg, "percent_complete"):
            feedback_msg.percent_complete = float(percent)
        if hasattr(feedback_msg, "status_text"):
            feedback_msg.status_text = str(text)
        if hasattr(feedback_msg, "status"):
            feedback_msg.status = str(text)
        if hasattr(feedback_msg, "message"):
            feedback_msg.message = str(text)

        try:
            goal_handle.publish_feedback(feedback_msg)
        except Exception:
            pass

    def _set_result_safe(self, result_msg, success: bool, text: str):
        """
        Populate whichever result fields exist in the installed action definition.
        """
        if hasattr(result_msg, "accepted"):
            # If execute callback ran, the goal was accepted by definition.
            result_msg.accepted = True
        if hasattr(result_msg, "success"):
            result_msg.success = bool(success)
        if hasattr(result_msg, "reason"):
            result_msg.reason = str(text)
        if hasattr(result_msg, "result_text"):
            result_msg.result_text = str(text)

    def _emit_topic_result(
        self,
        *,
        intent_id: str,
        command_id: str,
        success: bool,
        reason: str,
    ) -> None:
        payload = {
            "type": "playbook_result",
            "robot": self.robot,
            "intent_id": intent_id,
            "command_id": command_id,
            "success": bool(success),
            "reason": str(reason),
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.result_pub.publish(msg)

    def _execute_command_core(
        self,
        *,
        command_id: str,
        parameters_json: str,
        intent_id: str,
        north_m_typed: float = 0.0,
        east_m_typed: float = 0.0,
        feedback_cb=None,
        stop_event=None,
    ) -> Tuple[bool, str, Dict[str, object]]:
        """
        Shared command executor used by both:
        - Action interface (ExecutePlaybook)
        - Standalone topic interface (/execute_playbook_cmd)
        """
        if self._is_human_override_active():
            reason = "human teleop override active"
            self.get_logger().warn(f"[{self.robot}] rejecting autonomous command: {reason}")
            return False, reason, {}

        goal_start_time = time.time()

        ok, err, parsed = validate_and_normalize(command_id, parameters_json)
        if not ok or parsed is None:
            self.get_logger().warn(f"[{self.robot}] execute rejected late: {err}")
            try:
                raw_params = json.loads(parameters_json or "{}")
            except Exception:
                raw_params = {}
            self.audit.log_command(
                robot=self.robot,
                source="orchestrator",
                command_id=command_id,
                parameters=raw_params,
                status="failed",
                source_id=intent_id,
                details=err,
                duration_s=time.time() - goal_start_time,
            )
            return False, err, {}

        def fb(percent: float, text: str):
            if callable(feedback_cb):
                try:
                    feedback_cb(percent, text)
                except Exception:
                    pass

        # Log goal execution start
        self.audit.log_command(
            robot=self.robot,
            source="orchestrator",
            command_id=command_id,
            parameters=parsed.raw_params,
            status="started",
            source_id=intent_id,
        )

        # Apply speed scaling
        v = self.v * parsed.speed_scale
        vy = self.vy * parsed.speed_scale
        w = self.w * parsed.speed_scale
        turn_v = self.turn_v * parsed.speed_scale

        cid = parsed.command_id
        direction = parsed.direction
        duration_s = parsed.duration_s
        selected_strategy = ""

        if stop_event is None:
            stop_event = threading.Event()
        composite_stop = UnitExecutor._CompositeStopEvent(stop_event, self._human_override_event)

        try:
            if cid == "hold":
                plan = TimedTwistPlan(twist=Twist(), duration_s=duration_s, status_text="holding")
                run_timed_twist(self._publish, fb, plan, rate_hz=10.0, stop_at_end=True, stop_event=composite_stop)

            elif cid == "rotate":
                twist = Twist()
                twist.angular.z = w if direction in ("left", "ccw") else -w
                plan = TimedTwistPlan(twist=twist, duration_s=duration_s, status_text=f"rotate {direction}")
                run_timed_twist(self._publish, fb, plan, rate_hz=20.0, stop_at_end=True, stop_event=composite_stop)

            elif cid == "transit":
                twist = Twist()
                twist.linear.x = -v if direction in ("backward", "back", "-x") else +v
                plan = TimedTwistPlan(twist=twist, duration_s=duration_s, status_text=f"transit {direction}")
                run_timed_twist(self._publish, fb, plan, rate_hz=20.0, stop_at_end=True, stop_event=composite_stop)

            elif cid == "strafe":
                twist = Twist()
                if self.drive_type == "mecanum":
                    twist.linear.y = +vy if direction in ("left", "+y") else -vy
                plan = TimedTwistPlan(twist=twist, duration_s=duration_s, status_text=f"strafe {direction}")
                run_timed_twist(self._publish, fb, plan, rate_hz=20.0, stop_at_end=True, stop_event=composite_stop)

            elif cid == "diagonal":
                twist = Twist()
                if self.drive_type == "mecanum":
                    if direction == "fwd_left":
                        twist.linear.x, twist.linear.y = +v, +vy
                    elif direction == "fwd_right":
                        twist.linear.x, twist.linear.y = +v, -vy
                    elif direction == "back_left":
                        twist.linear.x, twist.linear.y = -v, +vy
                    else:
                        twist.linear.x, twist.linear.y = -v, -vy
                plan = TimedTwistPlan(twist=twist, duration_s=duration_s, status_text=f"diagonal {direction}")
                run_timed_twist(self._publish, fb, plan, rate_hz=20.0, stop_at_end=True, stop_event=composite_stop)

            elif cid == "turn":
                twist = Twist()
                if self.drive_type == "diff_drive":
                    twist.linear.x = turn_v
                    twist.angular.z = w if direction in ("left", "ccw") else -w
                    status = f"turn {direction}"
                else:
                    twist.angular.z = w if direction in ("left", "ccw") else -w
                    status = f"rotate {direction}"
                plan = TimedTwistPlan(twist=twist, duration_s=duration_s, status_text=status)
                run_timed_twist(self._publish, fb, plan, rate_hz=20.0, stop_at_end=True, stop_event=composite_stop)

            elif cid == "transit_xy":
                north_m = float(parsed.north_m)
                east_m = float(parsed.east_m)
                if abs(north_m) < 1e-9:
                    north_m = float(north_m_typed)
                if abs(east_m) < 1e-9:
                    east_m = float(east_m_typed)

                heading_rad = None
                if self.has_magnetometer:
                    heading_rad = float(self._latest_heading_rad)

                compiled = compile_transit_xy_plans(
                    drive_type=self.drive_type,
                    hardware=self.hardware,
                    north_m=north_m,
                    east_m=east_m,
                    v_fwd=v,
                    v_strafe=vy,
                    w_rot=w,
                    cardinal_mode=self.cardinal_mode,
                    heading_rad=heading_rad,
                )
                selected_strategy = compiled.strategy_id
                self.get_logger().info(
                    f"[{self.robot}] transit_xy north={north_m:.3f}m east={east_m:.3f}m "
                    f"cardinal_mode={self.cardinal_mode} strategy={selected_strategy} steps={len(compiled.plans)}"
                )
                self._run_plan_sequence(compiled.plans, fb, composite_stop)
        finally:
            # Safety brake: publish zero twist after every command execution path.
            # This avoids lingering motion if the last non-zero command persists
            # in downstream transport timing edge-cases.
            try:
                self._publish(Twist())
                time.sleep(0.02)
                self._publish(Twist())
            except Exception:
                pass

        reason = f"ok ({selected_strategy})" if selected_strategy else "ok"
        goal_duration_s = time.time() - goal_start_time
        self.audit.log_command(
            robot=self.robot,
            source="orchestrator",
            command_id=command_id,
            parameters=parsed.raw_params,
            status="succeeded",
            source_id=intent_id,
            duration_s=goal_duration_s,
        )
        return True, reason, {"strategy": selected_strategy}

    def _command_topic_cb(self, msg: String) -> None:
        """
        Standalone command interface (no fleet_orchestrator_interfaces required).
        """
        try:
            payload = json.loads(msg.data or "{}")
        except Exception:
            self.get_logger().warning(f"[{self.robot}] invalid command topic JSON")
            return

        command_id = str(payload.get("command_id", "")).strip()
        intent_id = str(payload.get("intent_id", f"topic_{int(time.time())}")).strip()
        parameters_json = payload.get("parameters_json")
        if not isinstance(parameters_json, str):
            params = dict(payload.get("parameters", {}) or {})
            parameters_json = json.dumps(params, separators=(",", ":"))

        north_m = float(payload.get("north_m", 0.0))
        east_m = float(payload.get("east_m", 0.0))
        stop_event = threading.Event()
        key = f"topic:{intent_id}"
        self._active_goals[key] = stop_event

        success, reason, _meta = self._execute_command_core(
            command_id=command_id,
            parameters_json=parameters_json,
            intent_id=intent_id,
            north_m_typed=north_m,
            east_m_typed=east_m,
            feedback_cb=None,
            stop_event=stop_event,
        )
        self._active_goals.pop(key, None)
        self._emit_topic_result(
            intent_id=intent_id,
            command_id=command_id,
            success=success,
            reason=reason,
        )

    # ---------------- executor ----------------
    def execute_cb(self, goal_handle):
        """
        Execute a validated goal by publishing a timed Twist.
        """
        if not HAS_FLEET_ACTION:
            # Defensive guard: action callback should not be active without action type.
            return None
        goal = goal_handle.request
        feedback = ExecutePlaybook.Feedback()
        intent_id = goal.intent_id if hasattr(goal, "intent_id") else "unknown"

        def fb(percent: float, text: str):
            self._publish_feedback_safe(goal_handle, feedback, percent, text)

        # Create a stop event so cancel requests can interrupt run_timed_twist
        stop_event = threading.Event()
        self._active_goals[id(goal_handle)] = stop_event
        success, reason, _meta = self._execute_command_core(
            command_id=str(goal.command_id),
            parameters_json=str(goal.parameters_json),
            intent_id=str(intent_id),
            north_m_typed=float(getattr(goal, "north_m", 0.0)),
            east_m_typed=float(getattr(goal, "east_m", 0.0)),
            feedback_cb=fb,
            stop_event=stop_event,
        )

        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        result = ExecutePlaybook.Result()
        self._set_result_safe(result, success=success, text=reason)
        self._active_goals.pop(id(goal_handle), None)
        return result

    def destroy_node(self) -> None:
        """Clean up resources and close audit log."""
        if hasattr(self, 'audit') and self.audit:
            self.audit.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UnitExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
