#!/usr/bin/env python3
# SPDX-License-Identifier: LicenseRef-Proprietary

"""
playbook_strategies.py

Profile-driven playbook strategy compiler.

This module turns platform-agnostic intent targets (north/east displacement)
into timed Twist plans that are specific to drive_type + hardware profile.

Important:
- API contract stays in cardinal terms (north/east) for fleet-level consistency.
- On robots without heading sensors, north/east is intentionally treated as
  body-frame forward/right fallback.
- On robots with heading sensors, north/east can be converted to body frame.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import List, Optional, Tuple

from geometry_msgs.msg import Twist

from .playbook_helpers import TimedTwistPlan


@dataclass(frozen=True)
class CompiledStrategy:
    strategy_id: str
    plans: List[TimedTwistPlan]


def _is_mecanum(drive_type: str) -> bool:
    dt = str(drive_type or "").strip().lower()
    return dt in ("mecanum", "omni", "omnidirectional", "mecanum_drive", "mecanum-drive")


def _strategy_id_for(drive_type: str, hardware: str) -> str:
    hw = str(hardware or "").strip().lower()
    if _is_mecanum(drive_type):
        # Current field behavior: mecanum lateral motion can be unreliable,
        # so we use rotate+forward axis stepping for predictable outcomes.
        return "omni_axis_turn_xy"
    if "l298n" in hw:
        return "diff_axis_split_xy"
    if "tb6612" in hw:
        return "diff_heading_then_transit_xy"
    return "diff_heading_then_transit_xy"


def _mk_twist(vx: float = 0.0, vy: float = 0.0, wz: float = 0.0) -> Twist:
    tw = Twist()
    tw.linear.x = float(vx)
    tw.linear.y = float(vy)
    tw.angular.z = float(wz)
    return tw


def _duration(distance: float, speed: float) -> float:
    s = max(1e-6, abs(float(speed)))
    return abs(float(distance)) / s


def _global_ne_to_body_fr(north_m: float, east_m: float, heading_rad: float) -> Tuple[float, float]:
    """
    Convert global cardinal displacement (north/east) to body-frame (forward/right).

    heading_rad:
      robot yaw relative to north; 0 means robot forward points north.
    """
    c = math.cos(heading_rad)
    s = math.sin(heading_rad)
    forward_m = north_m * c + east_m * s
    right_m = -north_m * s + east_m * c
    return forward_m, right_m


def _resolve_body_displacement(
    north_m: float,
    east_m: float,
    cardinal_mode: str,
    heading_rad: Optional[float],
) -> Tuple[float, float]:
    """
    Resolve body-frame displacement from cardinal request.

    Modes:
      - relative_fallback: interpret north/east as forward/right (no magnetometer)
      - global_cardinal: convert from north/east using heading_rad
    """
    mode = str(cardinal_mode or "").strip().lower()
    if mode == "global_cardinal" and heading_rad is not None:
        return _global_ne_to_body_fr(north_m, east_m, float(heading_rad))
    return north_m, east_m


def _plan_omni_axis_turn(forward_m: float, right_m: float, v_fwd: float, w_rot: float) -> List[TimedTwistPlan]:
    if abs(forward_m) < 1e-9 and abs(right_m) < 1e-9:
        return [TimedTwistPlan(twist=Twist(), duration_s=0.0, status_text="already at target")]
    plans: List[TimedTwistPlan] = []

    # Execute objective in two axis-aligned legs:
    #   1) forward/backward leg in current heading
    #   2) rotate +/- 90 degrees and drive the right/left leg forward
    # This avoids dependence on strafing behavior.
    if abs(forward_m) > 1e-9:
        plans.append(
            TimedTwistPlan(
                twist=_mk_twist(vx=abs(v_fwd) if forward_m > 0.0 else -abs(v_fwd), vy=0.0, wz=0.0),
                duration_s=_duration(forward_m, v_fwd),
                status_text=f"omni forward leg {forward_m:.2f}m",
            )
        )

    if abs(right_m) > 1e-9:
        quarter_turn = math.pi / 2.0
        to_right_sign = -1.0 if right_m > 0.0 else +1.0
        plans.append(
            TimedTwistPlan(
                twist=_mk_twist(vx=0.0, vy=0.0, wz=to_right_sign * abs(w_rot)),
                duration_s=quarter_turn / max(1e-6, abs(w_rot)),
                status_text="omni rotate to lateral leg",
            )
        )
        plans.append(
            TimedTwistPlan(
                twist=_mk_twist(vx=abs(v_fwd), vy=0.0, wz=0.0),
                duration_s=abs(right_m) / max(1e-6, abs(v_fwd)),
                status_text=f"omni lateral leg right={right_m:.2f}m",
            )
        )

    return plans


def _plan_diff_heading_then_transit(forward_m: float, right_m: float, v_fwd: float, w_rot: float) -> List[TimedTwistPlan]:
    plans: List[TimedTwistPlan] = []
    distance = math.hypot(forward_m, right_m)
    if distance < 1e-9:
        return [TimedTwistPlan(twist=Twist(), duration_s=0.0, status_text="already at target")]

    theta = math.atan2(-right_m, forward_m)  # +theta means rotate left (CCW)
    if abs(theta) > 1e-3:
        plans.append(
            TimedTwistPlan(
                twist=_mk_twist(wz=abs(w_rot) if theta > 0.0 else -abs(w_rot)),
                duration_s=abs(theta) / max(1e-6, abs(w_rot)),
                status_text=f"rotate to heading {math.degrees(theta):.1f}deg",
            )
        )

    plans.append(
        TimedTwistPlan(
            twist=_mk_twist(vx=abs(v_fwd)),
            duration_s=distance / max(1e-6, abs(v_fwd)),
            status_text=f"transit distance {distance:.2f}m",
        )
    )
    return plans


def _plan_diff_axis_split(forward_m: float, right_m: float, v_fwd: float, w_rot: float) -> List[TimedTwistPlan]:
    """
    Axis-split strategy (no heading restore):
      1) complete forward/backward leg first
      2) rotate +/- 90 deg toward lateral leg
      3) complete right/left leg as forward motion
    """
    plans: List[TimedTwistPlan] = []
    if abs(forward_m) < 1e-9 and abs(right_m) < 1e-9:
        return [TimedTwistPlan(twist=Twist(), duration_s=0.0, status_text="already at target")]

    if abs(forward_m) > 1e-9:
        plans.append(
            TimedTwistPlan(
                twist=_mk_twist(vx=abs(v_fwd) if forward_m > 0.0 else -abs(v_fwd)),
                duration_s=abs(forward_m) / max(1e-6, abs(v_fwd)),
                status_text=f"axis-split forward leg {forward_m:.2f}m",
            )
        )

    if abs(right_m) > 1e-9:
        quarter_turn = math.pi / 2.0
        to_right_sign = -1.0 if right_m > 0.0 else +1.0
        plans.append(
            TimedTwistPlan(
                twist=_mk_twist(wz=to_right_sign * abs(w_rot)),
                duration_s=quarter_turn / max(1e-6, abs(w_rot)),
                status_text="axis-split rotate to lateral leg",
            )
        )
        plans.append(
            TimedTwistPlan(
                twist=_mk_twist(vx=abs(v_fwd)),
                duration_s=abs(right_m) / max(1e-6, abs(v_fwd)),
                status_text=f"axis-split lateral leg right={right_m:.2f}m",
            )
        )
    return plans


def compile_transit_xy_plans(
    drive_type: str,
    hardware: str,
    north_m: float,
    east_m: float,
    v_fwd: float,
    v_strafe: float,
    w_rot: float,
    cardinal_mode: str = "relative_fallback",
    heading_rad: Optional[float] = None,
) -> CompiledStrategy:
    """
    Compile an XY displacement into one or more timed Twist plans.

    cardinal_mode:
      - relative_fallback: interpret north/east as forward/right
      - global_cardinal: convert north/east with heading sensor
    """
    forward_m, right_m = _resolve_body_displacement(
        north_m=north_m,
        east_m=east_m,
        cardinal_mode=cardinal_mode,
        heading_rad=heading_rad,
    )
    sid = _strategy_id_for(drive_type, hardware)
    if sid == "omni_axis_turn_xy":
        plans = _plan_omni_axis_turn(forward_m, right_m, v_fwd, w_rot)
    elif sid == "diff_axis_split_xy":
        plans = _plan_diff_axis_split(forward_m, right_m, v_fwd, w_rot)
    else:
        plans = _plan_diff_heading_then_transit(forward_m, right_m, v_fwd, w_rot)
    return CompiledStrategy(strategy_id=sid, plans=plans)
