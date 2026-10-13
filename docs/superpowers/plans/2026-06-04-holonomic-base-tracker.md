# Holonomic Mecanum Base Tracker — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace non-holonomic control law in `gazebo_base_tracker.py` with holonomic one, enabling mecanum-wheeled robot to use lateral motion for trajectory tracking.

**Architecture:** Single-file edit in `gazebo_base_tracker.py`. Change gains, replace feedforward velocity projection, add `cmd.linear.y` output, decouple angular control from lateral error.

**Tech Stack:** Python 3, ROS geometry_msgs/Twist

---

### Task 1: Apply holonomic control law

**Files:**
- Modify: `src/mm_gazebo/scripts/gazebo_base_tracker.py`

- [ ] **Step 1: Update gains (lines 28-30)**

Replace:
```python
        self.k_x = 1.0
        self.k_y = 2.0
        self.k_yaw = 2.0
```

With:
```python
        self.k_x = 1.5
        self.k_y = 1.5
        self.k_yaw = 2.0
```

- [ ] **Step 2: Replace control law in `control_loop` (lines 64-94)**

Replace the entire `control_loop` method body with:

```python
    def control_loop(self, event):
        if not self.has_odom or not self.has_target:
            return

        # Errors in global frame
        ex_global = self.target_x - self.current_x
        ey_global = self.target_y - self.current_y

        # Errors in local frame
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

        # limits
        if cmd.linear.x > 3.0: cmd.linear.x = 3.0
        if cmd.linear.x < -3.0: cmd.linear.x = -3.0
        if cmd.angular.z > 2.0: cmd.angular.z = 2.0
        if cmd.angular.z < -2.0: cmd.angular.z = -2.0

        self.cmd_pub.publish(cmd)
```

- [ ] **Step 3: Verify Python syntax**

```bash
python3 -m py_compile src/mm_gazebo/scripts/gazebo_base_tracker.py
```

Expected: No output (compiles cleanly).

- [ ] **Step 4: Launch simulation and verify tracking**

```bash
roslaunch mm_remani exp0_gazebo.launch
```

Expected: Compare `/remani_planner_node/optimal_ctrl_pts` (optimized trajectory) with `/model_vis/mm_his_traj` (actual path) in RViz. Curve tracking should show tighter adherence. Straight-line behavior unchanged.
