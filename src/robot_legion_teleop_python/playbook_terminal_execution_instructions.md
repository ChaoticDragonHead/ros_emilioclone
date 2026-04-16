# Playbook Terminal Execution Instructions

This guide shows how to run playbooks from a laptop terminal against multiple robots, starting from fresh terminals and fresh SSH sessions.

Assumptions:
- ROS 2 Jazzy on Ubuntu 24.04.
- Robot name is derived from Linux username (`$USER`) by `robot_bringup.launch.py`.
- Linux username on each robot should match a key in `config/robot_profiles.yaml` (for example: `robot1`, `robot2`, `robot3`).
- All machines are on the same network and use the same ROS domain ID.

Standalone note:
- `robot_legion_teleop_python` now runs without `fleet_orchestrator_interfaces`.
- If interfaces are present, it uses action transport (`/execute_playbook`).
- If interfaces are absent, it automatically uses standalone topic transport (`/execute_playbook_cmd`).

## 1) On each robot Pi (fresh SSH session)

Open one SSH terminal per robot, then run:

```bash
cd "$HOME/ros2_ws"
source /opt/ros/"${ROS_DISTRO:-jazzy}"/setup.bash
colcon build --packages-select robot_legion_teleop_python
source "$HOME/ros2_ws/install/setup.bash"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export PROFILES_PATH="${PROFILES_PATH:-$HOME/ros2_ws/src/robot_legion_teleop_python/config/robot_profiles.yaml}"
```

Important:
- If a shell already has `ROS_DOMAIN_ID=17`, the `:-42` pattern will keep `17`.
- To force consistent values every time, use this instead:

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export PROFILES_PATH="$HOME/ros2_ws/src/robot_legion_teleop_python/config/robot_profiles.yaml"
```

Note:
- `ROBOT_NAME` is not consumed by current `robot_bringup.launch.py`.
- Runtime robot namespace/action path is based on Linux username (`$USER`), e.g. `/<user>/execute_playbook`.
- If `$USER` does not exist in `robot_profiles.yaml`, motor driver may fall back to mock mode and the robot will not physically move.

Verify robot shell environment:

```bash
env | grep -E 'ROS_DOMAIN_ID|ROS_LOCALHOST_ONLY|ROS_AUTOMATIC_DISCOVERY_RANGE|ROS_STATIC_PEERS|RMW_IMPLEMENTATION|ROS_DISTRO|USER'
```

Launch robot nodes (motor driver, heartbeat, executor):

```bash
ros2 launch robot_legion_teleop_python robot_bringup.launch.py \
  profiles_path:="$PROFILES_PATH" \
  use_camera:=false
```

Expected in logs:
- `UnitExecutor ready`
- `action=/<robot_name>/execute_playbook` (if `fleet_orchestrator_interfaces` is installed)
- `cmd_iface=/<robot_name>/execute_playbook_cmd` (always, standalone-safe)

## 2) On laptop (fresh terminal)

```bash
cd "$HOME/ros2_ws"
source /opt/ros/"${ROS_DISTRO:-jazzy}"/setup.bash
colcon build --packages-select robot_legion_teleop_python
source "$HOME/ros2_ws/install/setup.bash"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
```

To force known-good values (recommended), run:

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

Verify laptop shell environment:

```bash
env | grep -E 'ROS_DOMAIN_ID|ROS_LOCALHOST_ONLY|ROS_AUTOMATIC_DISCOVERY_RANGE|ROS_STATIC_PEERS|RMW_IMPLEMENTATION|ROS_DISTRO|USER'
```

Discover action servers:

```bash
mapfile -t ACTIONS < <(ros2 action list | grep -E '^/.+/execute_playbook$')
echo "Found ${#ACTIONS[@]} execute_playbook servers"
printf '%s\n' "${ACTIONS[@]}"
```

Discover standalone command topics (works without fleet interfaces):

```bash
ros2 topic list | grep -E '^/.+/execute_playbook_cmd$'
```

If count is `0`, check:
- Same `ROS_DOMAIN_ID` on laptop and all robots.
- Robot launch logs show executor started.

## 3) Run `terminal_orchestrator.py` (recommended)

Start the terminal UI:

```bash
ros2 run robot_legion_teleop_python terminal_orchestrator
```

You should see:
- `Detected robots: <N>`
- robot list under `Robots:`

Detection behavior:
- `Detected robots` now means **reachable** robots, not just graph-discovered names.
- `terminal_orchestrator` supports two transports:
  - action: `/<robot>/execute_playbook` (when `fleet_orchestrator_interfaces` exists)
  - standalone topic: `/<robot>/execute_playbook_cmd` (always available)
- If action transport exists, it is preferred.
- Probe timeout parameter:
  - `reachable_probe_timeout_s` (default `0.15`)

UI controls:
- `t` choose target robots (`all` or one robot)
- `1` Detected-robot XY planner (deterministic linear algebra)
- `2` Execute ALL commands (validation sweep)
- `3` Transit distance (meters -> duration estimate)
- `4` Rotate degrees (degrees -> duration estimate)
- `s` Playbook sequence (queue multiple playbooks before running)
- `!` STOP ALL ROBOTS (best-effort software stop)
- `r` refresh discovery
- `q` quit

Playbook sequence submenu:
- Header shows `EXECUTE PLAYBOOK SEQUENCE` (separate from the main menu header).
- Lets you queue from the same playbooks (`1/2/3/4`) and run them in order.
- Queue controls:
  - `l` load queue from JSON file
  - `n` save current queue as a new preset JSON file
  - `e` execute queued playbooks
  - `d` delete last queued playbook
  - `c` clear queue
  - `b` back
  - `u` update queued playbook by entry number
  - `x` delete queued playbook by entry number
  - `!` STOP ALL ROBOTS
- Sequence execute behavior (`e`):
  - non-interactive per step (no per-step pause, no per-step confirmation).
  - one final `SEQUENCE SUMMARY` screen after the queue run.
  - one final pretty-printed (indented) JSON block for copy/paste.
- JSON queue loading (`l`):
  - Enter a preset **name** (not full path).
  - Press Enter to use default (`fleet_preset_sequence_patrol`).
  - Enter `?` to list available preset JSON files.
  - `.json` suffix is optional; it is added automatically.
  - Choose load mode:
    - `replace`: replace current queue with file content
    - `append`: append file content to current queue
  - Supported JSON top-level shapes:
    - Fleet envelope: `{ "type": "fleet_playbook_request", "sequence": [ ... ] }`
    - Lightweight local: `{ "sequence": [ ... ] }` or `{ "queue": [ ... ] }`
  - Each step object must contain:
    - `playbook`: one of `move_xy`, `execute_all_commands`, `transit_distance`, `rotate_degrees`
    - `params`: playbook parameters
    - optional `label`
- Saving a queue (`n`):
  - Writes current `Queued:` list to a readable, indented JSON preset file.
  - Prompt asks for preset **name**.
  - Default name:
    - `fleet_preset_custom`
  - File is saved under:
    - `~/ros2_ws/src/robot_legion_teleop_python/config/presets/`
  - `.json` suffix is optional; it is added automatically.
  - Saved files use the same fleet-ready envelope (`type: fleet_playbook_request`) so they can be loaded later with `l`.
- Playbook `1` in sequence mode:
  - fully queueable now.
  - captures playbook-1 variables either from prompts or JSON:
    - `x`, `y`, `speed`, `MAX PATH OFFSET`,
    - main robot selection,
    - per-robot heading config (`heading_clock`).
  - executes non-interactively during sequence run using those captured values.
  - If JSON contains stale/missing `detected_robots` or invalid `main_robot`, runtime auto-resolves from currently reachable robots.

Execution flow:
1. Choose target robots.
2. Choose a playbook.
3. Enter requested values.
4. Review `EXECUTION SUMMARY`.
5. Confirm with `y` to dispatch.

Notes:
- Dispatch is sent to all selected robots in parallel (simultaneous start intent).
- Distance/time is intentionally approximate and consistent across robots.
- For `Move objective (x, y)`: `x` maps to forward/back, `y` maps to right/left.
- Playbook `1` specifically uses all detected robots (it does not limit to the currently selected target subset).
- Dispatch-time skip policy: if a robot blocks dispatch for more than `1.0 s`, it is skipped with explicit timeout text in output.

## 4) Manual CLI playbook commands (alternative)

### Example A: all robots move forward (transit primitive)

What it does:
- Sends `command_id: transit` to each discovered robot.
- Uses `direction=forward`, `duration_s=2.0`, `speed=0.8`.

```bash
for action in "${ACTIONS[@]}"; do
  robot="${action%/execute_playbook}"; robot="${robot#/}"
  ros2 action send_goal --feedback "$action" fleet_orchestrator_interfaces/action/ExecutePlaybook \
    "{intent_id: test_transit_${robot}, command_id: transit, parameters_json: \"{\\\"direction\\\":\\\"forward\\\",\\\"duration_s\\\":2.0,\\\"speed\\\":0.8}\"}" &
done
wait
```

### Example B: all robots move backward (same primitive, reverse direction)

What it does:
- Same as Example A, but `direction=backward`.

```bash
for action in "${ACTIONS[@]}"; do
  robot="${action%/execute_playbook}"; robot="${robot#/}"
  ros2 action send_goal --feedback "$action" fleet_orchestrator_interfaces/action/ExecutePlaybook \
    "{intent_id: test_transit_back_${robot}, command_id: transit, parameters_json: \"{\\\"direction\\\":\\\"backward\\\",\\\"duration_s\\\":2.0,\\\"speed\\\":0.8}\"}" &
done
wait
```

### Example C: all robots move forward, then backward after forward completes

What it does:
- First dispatches forward transit to all robots and waits for completion.
- Then dispatches backward transit to all robots.

```bash
for action in "${ACTIONS[@]}"; do
  (
    robot="${action%/execute_playbook}"; robot="${robot#/}"

    # forward for this robot
    ros2 action send_goal --feedback "$action" fleet_orchestrator_interfaces/action/ExecutePlaybook \
      "{intent_id: test_seq_fwd_${robot}, command_id: transit, parameters_json: \"{\\\"direction\\\":\\\"forward\\\",\\\"duration_s\\\":2.0,\\\"speed\\\":0.8}\"}"

    # then backward for this same robot
    ros2 action send_goal --feedback "$action" fleet_orchestrator_interfaces/action/ExecutePlaybook \
      "{intent_id: test_seq_back_${robot}, command_id: transit, parameters_json: \"{\\\"direction\\\":\\\"backward\\\",\\\"duration_s\\\":2.0,\\\"speed\\\":0.8}\"}"
  ) &
done
wait

```

### Example D: all robots rotate left ~45 degrees, then move forward 1.5 seconds

What it does:
- Step 1: rotate left.
- Step 2: move forward.

Notes:
- Rotation command is duration-based, not angle-based.
- With default rotate speed `2.0 rad/s`, 45 degrees (`pi/4`) is about `0.39 s`.

```bash
# Step 1: rotate left ~45 deg (duration-based approximation)
for action in "${ACTIONS[@]}"; do
  robot="${action%/execute_playbook}"; robot="${robot#/}"
  ros2 action send_goal --feedback "$action" fleet_orchestrator_interfaces/action/ExecutePlaybook \
    "{intent_id: test_rot45_${robot}, command_id: rotate, parameters_json: \"{\\\"direction\\\":\\\"left\\\",\\\"duration_s\\\":0.39,\\\"speed\\\":1.0}\"}" &
done
wait

# Step 2: move forward 1.5 s
for action in "${ACTIONS[@]}"; do
  robot="${action%/execute_playbook}"; robot="${robot#/}"
  ros2 action send_goal --feedback "$action" fleet_orchestrator_interfaces/action/ExecutePlaybook \
    "{intent_id: test_after_rot_fwd_${robot}, command_id: transit, parameters_json: \"{\\\"direction\\\":\\\"forward\\\",\\\"duration_s\\\":1.5,\\\"speed\\\":0.8}\"}" &
done
wait
```

### Example E: all robots move to an objective point (`transit_xy`)

What it does:
- Sends the same objective-point command to each robot (`north_m`, `east_m`).
- Robot-specific motion strategy is selected robot-side from drive/hardware profile.
- Current field strategy executes axis legs using forward+rotate behavior (no strafe dependency):
  first leg, then ~90 deg rotate, then second leg.

Notes:
- `east_m > 0` means objective is to robot-right.
- `east_m < 0` means objective is to robot-left.

```bash
for action in "${ACTIONS[@]}"; do
  robot="${action%/execute_playbook}"; robot="${robot#/}"
  ros2 action send_goal --feedback "$action" fleet_orchestrator_interfaces/action/ExecutePlaybook \
    "{intent_id: test_objective_xy_${robot}, command_id: transit_xy, parameters_json: \"{\\\"north_m\\\":1.5,\\\"east_m\\\":-0.75,\\\"speed\\\":1.0}\"}" &
done
wait
```

Design guidance:
- Keep terminal commands platform-agnostic when possible (single `transit_xy` contract).
- Put per-drive behavior (for example, mecanum forward-then-strafe vs differential axis/heading strategy) in playbook strategy code, not shell branching.

## 5) Optional: monitor action and robot state

On laptop:

```bash
ros2 action list | grep execute_playbook
```

On a robot terminal:

```bash
tail -f /tmp/robot_"${USER}"_audit.jsonl
```

## 6) Interpreting `terminal_orchestrator` JSON output

After you confirm `EXECUTE? y/n`, `terminal_orchestrator` prints structured JSON events.
These are intended to be copy/paste friendly for future Fleet Orchestrator integration work.

Event types you will see:
- `type: "playbook_command"`: dispatch intent per robot (task-style event).
- `type: "audit_event"`: execution outcome per robot (result-style event).

For option `1` specifically, you will see:
- one high-level planning event with `command_id: "transit_xy_deterministic_formation"`
- then per-phase dispatch events (`rotate`, `transit`, `rotate`, `transit`) for each robot
- then per-phase `audit_event` results for each robot

### A) Task-style dispatch event (`playbook_command`)

Use this to verify command mapping before execution:
- `intent_id`: unique command instance ID.
- `command_id`: primitive being sent (`transit`, `rotate`, `transit_xy`, ...).
- `targets`: robot receiving the command.
- `parameters`: normalized command values sent to robot executor.
- `preview.strategy_id`: strategy selected for that robot type.
- `preview.steps`: ordered movement phases (direction + duration).

Example:

```json
{
  "command_id": "transit_xy",
  "intent_id": "term_xy_robot2_1770000000",
  "parameters": {
    "east_m": 0.5,
    "north_m": 0.2,
    "speed": 1.0
  },
  "platform": {
    "adapter_type": "terminal_orchestrator",
    "platform_family": "robot_legion_teleop_python"
  },
  "preview": {
    "cardinal_mode": "relative_fallback",
    "drive_type": "mecanum_drive",
    "hardware": "dual_tb6612_mecanum",
    "notes": [
      "north/east uses robot body frame without heading sensor",
      "robots with different starting headings diverge in world frame"
    ],
    "profile_name": "mecanum_drive",
    "profile_source": "registry",
    "robot": "robot2",
    "steps": [
      {
        "duration_s": 0.5,
        "index": 1,
        "motion": "forward",
        "status_text": "omni forward leg 0.20m"
      },
      {
        "duration_s": 0.785,
        "index": 2,
        "motion": "rotate_right",
        "status_text": "omni rotate to lateral leg"
      },
      {
        "duration_s": 1.25,
        "index": 3,
        "motion": "forward",
        "status_text": "omni lateral leg right=0.50m"
      }
    ],
    "strategy_id": "omni_axis_turn_xy"
  },
  "targets": "robot2",
  "trace": {
    "confidence": 1.0,
    "explanation": "manual terminal playbook dispatch",
    "translator": "terminal_orchestrator"
  },
  "type": "playbook_command"
}
```

### B) Audit-style result event (`audit_event`)

Use this to verify execution completion:
- `status`: `succeeded` or `failed`.
- `accepted`: whether the robot action server accepted execution.
- `reason`: robot-side result reason string.
- `intent_id`: links result back to the original dispatch event.

Example:

```json
{
  "accepted": true,
  "intent_id": "term_xy_robot2_1770000000",
  "reason": "ok (omni_axis_turn_xy)",
  "robot": "robot2",
  "stage": "execute_playbook_dispatch",
  "status": "succeeded",
  "type": "audit_event"
}
```

### C) Optional topic publishing mode

By default, JSON is printed to terminal only.
To also publish JSON to Fleet-Orchestrator-style topics:

```bash
ros2 run robot_legion_teleop_python terminal_orchestrator --ros-args -p publish_fleet_topics:=true
```

Then monitor:

```bash
ros2 topic echo /fo/task
ros2 topic echo /fo/audit
```

## 6.1 Emergency stop behavior

`terminal_orchestrator` includes a global software stop command:
- `!`, `stop`, `k`, `kill`, `5`, or `space` in menu/prompt contexts triggers `STOP ALL ROBOTS`.
- It dispatches a short `hold` command (`duration_s: 0.3`) to all reachable robots.
- During active motion/spin waits, terminal input is line-buffered:
  enter a stop token and press Enter.
- Available from:
  - main menu
  - target select menu
  - execute playbook sequence menu
  - numeric prompts
  - execution confirm prompt
- After stop:
  - from main/playbook flows: returns to main menu
  - from sequence flow: returns to `EXECUTE PLAYBOOK SEQUENCE`

## 7) Detected-Robot Coordinated XY mode (option `1`)

`1) Move to objective` runs a coordinated planner over all currently detected robots.

What it does:
1. Uses all detected robots.
2. Asks for:
   - base objective (`x`, `y`)
     - defaults: `x=1.0`, `y=1.0` (non-zero 2D default objective)
   - speed scale
   - main robot selection (anchor/reference robot)
   - initial heading confirmation step for non-main robots:
     - default: all non-main robots are assumed to face the same direction as main (`12 o'clock`)
     - user can customize heading clock per non-main robot
   - `MAX PATH OFFSET m` (default `0.8`)
3. Uses deterministic linear algebra to compute trajectories:
   - all robots share the SAME objective point `(x, y)`
   - main robot: one rotation + one forward/back movement (shortest straight path)
   - non-main robots: two rotations + two forward/back movements (V-shaped path)
   - non-main turn side is auto-selected from destination geometry:
     - if `dy > 0` (target to robot-right), planner prefers `left` side
     - if `dy < 0` (target to robot-left), planner prefers `right` side
     - if `dy ~= 0`, planner uses alternating hint (`left,right,left,...`)
     - if preferred side is infeasible, planner retries opposite side automatically
   - core decomposition logic for non-main robots:
     - compute two vectors `v1`, `v2` such that `v1 + v2 = destination_vector`
     - execute leg1 along `v1`, then leg2 along `v2`
     - this gives a deterministic V-path without requiring fallback
   - when custom heading is provided for non-main robots, planner converts
     the shared objective from main frame into each robot frame before solving
4. Enforces detour policy from the main robot straight path:
   - main robot detour target = `0`
   - second robot (next detected after main) detour target = `MAX PATH OFFSET`
   - additional robots get evenly spaced detour targets between `0` and `MAX PATH OFFSET`
5. Non-main final headings are not globally forced to be equal; they follow the
   solved V-path geometry.
6. Executes in synchronized phases:
   - phase 1: rotate
   - phase 2: forward leg 1
   - phase 3: rotate
   - phase 4: forward leg 2

### Linear algebra summary

For each robot, the planner solves:

```text
l1 * [cos(theta1), sin(theta1)] + l2 * [cos(theta2), sin(theta2)] = [dx, dy]
```

Where:
- `[dx, dy]` is that robot's destination vector in robot-relative frame.
- `v1 = l1*[cos(theta1), sin(theta1)]`
- `v2 = l2*[cos(theta2), sin(theta2)]`
- planner chooses `theta1/theta2` (side-aware) and `l1/l2` so `v1 + v2 = [dx,dy]`.

The planner rejects a candidate solution if:
- matrix is singular,
- any leg length is too small/non-forward,
- detour from main straight path exceeds `MAX PATH OFFSET m`.

Feasibility robustness:
- V-path decomposition is solved directly from vector addition, which removes
  most infeasibility cases seen with over-constrained heading coupling.

Important variables (playbook `1` and dispatch):
- `MAX PATH OFFSET m` (UI input, default `0.8`): max allowable detour from main robot straight path.
- per-robot heading input (optional override):
  - `heading_clock` relative to main robot (`12` means same direction)
- per-robot duration calibration (in `robot_profiles.yaml`, `robots.<name>.params.drive`):
  - `orchestrator_linear_duration_scale`
  - `orchestrator_angular_duration_scale`
  - terminal_orchestrator applies these per robot when converting meters/degrees to duration.
- side selection mode for non-main robots:
  - `auto_dy_sign_with_fallback` (preferred side from `dy`, fallback to opposite side if needed)
- final heading mode:
  - `non_main_v_path_variable_heading` (non-main robots end at V-path-derived heading)
- `dispatch_timeout_s` (node param, default `1.0`): wait limit for action server availability before skipping robot.
- `send_goal_response_timeout_s` (node param, default `1.0`): wait limit for goal-accept response before skipping robot.

Timeout skip behavior:
- If either dispatch timeout is exceeded, robot is skipped for that phase.
- Program output explicitly includes:
  - `skipped due to timeout: waited > 1.0s ...`
- JSON audit output includes:
  - `status: "skipped"`
  - `details` containing timeout cause.

### Important frame note

All vectors are command-executed robot-relative.
For playbook `1`, the objective is shared in main-robot frame and transformed
into each non-main robot frame using `heading_clock`.

Custom heading prompt format (per non-main robot):

```text
relative to the main robot (<main_robot>),
<robot> is pointing toward ___ o'clock
enter a value:
```

Example:

```text
relative to the main robot (robot2),
robot1 is pointing toward ___ o'clock
enter a value:
```
