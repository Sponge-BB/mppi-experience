# Holonomic Mecanum Base Tracker Control Law

## Date

2026-06-04

## Problem

In Gazebo simulation, the actual robot trajectory (`/model_vis/mm_his_traj`) deviates from the optimized polynomial trajectory (`/remani_planner_node/optimal_ctrl_pts`) during turns. Straight-line tracking is fine, but the robot cuts corners and lags behind on curved paths.

### Root Cause

`gazebo_base_tracker.py` uses a non-holonomic (differential-drive) control law on a holonomic (mecanum-wheeled) robot:

1. **No lateral velocity feedforward**: Global desired velocity is only projected onto the forward direction, discarding the lateral component.
2. **`cmd.linear.y` never set**: Mecanum wheels' sideways capability is unused.
3. **Lateral error corrected through yaw rotation**: The `k_y * v_r * ey` term in `cmd.angular.z` tries to fix lateral tracking error by turning the robot, which is slow and creates overshoot on curves.

### Why curves fail but straights work

- **Straight lines**: `target_yaw` constant, `eyaw ≈ 0`, forward feedforward sufficient.
- **Curves**: `target_yaw` rotates continuously, `eyaw` grows, the non-holonomic controller wastes time rotating instead of directly translating sideways. The mecanum robot physically CAN translate sideways, but the controller never commands it.

## Fix

Replace the non-holonomic control law with a holonomic one in `gazebo_base_tracker.py`.

### File

`src/mm_gazebo/scripts/gazebo_base_tracker.py`

### Change 1: Gains (lines 28-30)

```python
# Before
self.k_x = 1.0
self.k_y = 2.0
self.k_yaw = 2.0

# After
self.k_x = 1.5
self.k_y = 1.5
self.k_yaw = 2.0
```

### Change 2: Control law (lines 64-94)

```python
# Before
# Target local feedforward velocity
v_r = self.target_vx * math.cos(self.target_yaw) + self.target_vy * math.sin(self.target_yaw)

# Errors in global frame (unchanged)
ex_global = self.target_x - self.current_x
ey_global = self.target_y - self.current_y

# Errors in local frame (unchanged)
ex = math.cos(self.current_yaw) * ex_global + math.sin(self.current_yaw) * ey_global
ey = -math.sin(self.current_yaw) * ex_global + math.cos(self.current_yaw) * ey_global
eyaw = self.normalize_angle(self.target_yaw - self.current_yaw)

dist = math.hypot(ex_global, ey_global)

# Stop condition
if dist < 0.15 and abs(eyaw) < 0.1 and abs(v_r) < 0.05:
    cmd = Twist()
    self.cmd_pub.publish(cmd)
    return

cmd = Twist()

# Smooth trajectory tracking control law for non-holonomic mobile robots
cmd.linear.x = v_r * math.cos(eyaw) + self.k_x * ex
cmd.angular.z = self.k_yaw * eyaw + self.k_y * v_r * ey

# After
# Errors in global frame (unchanged)
ex_global = self.target_x - self.current_x
ey_global = self.target_y - self.current_y

# Errors in local frame (unchanged)
ex = math.cos(self.current_yaw) * ex_global + math.sin(self.current_yaw) * ey_global
ey = -math.sin(self.current_yaw) * ex_global + math.cos(self.current_yaw) * ey_global
eyaw = self.normalize_angle(self.target_yaw - self.current_yaw)

dist = math.hypot(ex_global, ey_global)

# Stop condition
if dist < 0.15 and abs(eyaw) < 0.1:
    cmd = Twist()
    self.cmd_pub.publish(cmd)
    return

# Holonomic feedforward: project global desired velocity to robot body frame
vx_des = self.target_vx * math.cos(self.current_yaw) + self.target_vy * math.sin(self.current_yaw)
vy_des = -self.target_vx * math.sin(self.current_yaw) + self.target_vy * math.cos(self.current_yaw)

cmd = Twist()

# Holonomic control for mecanum-wheeled robot
cmd.linear.x = vx_des + self.k_x * ex
cmd.linear.y = vy_des + self.k_y * ey
cmd.angular.z = self.k_yaw * eyaw
```

### Change summary

| Aspect | Before | After |
|--------|--------|-------|
| Forward velocity | `v_r * cos(eyaw) + k_x * ex` | `vx_des + k_x * ex` |
| Lateral velocity | 0 (never set) | `vy_des + k_y * ey` |
| Angular velocity | `k_yaw * eyaw + k_y * v_r * ey` | `k_yaw * eyaw` |
| Feedforward | Forward only (projected onto target_yaw) | Full (projected onto current_yaw) |
| Constraint model | Non-holonomic (diff-drive) | Holonomic (mecanum) |
| Stop condition | Checks `abs(v_r) < 0.05` | Simplified, distance + yaw only |

## Scope

- File: `src/mm_gazebo/scripts/gazebo_base_tracker.py`
- ~15 lines changed
- No new files, no API changes, no config changes
- Python only, no C++ rebuild needed

## Testing

- Launch `exp0_gazebo.launch`, compare `/remani_planner_node/optimal_ctrl_pts` with `/model_vis/mm_his_traj` in RViz
- Verify straight-line tracking unchanged
- Verify curve tracking improved — trajectory should hug optimized path closely
- Verify no instability (no oscillation around target)
