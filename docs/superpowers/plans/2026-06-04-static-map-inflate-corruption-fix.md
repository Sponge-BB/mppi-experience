# Static Map Inflate Corruption Fix — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix 3 bugs in `grid_map.cpp` that cause static pre-built map occupancy to be destroyed when dynamic LiDAR obstacles trigger the occupancy update pipeline.

**Architecture:** Three surgical edits in `clearAndInflateLocalMap()` and `cloudCallback()` of `grid_map.cpp`. Each edit adds a `static_occupancy_` guard or corrects data initialization. No new files, no API changes.

**Tech Stack:** C++ (ROS), `std::vector`, Eigen

---

### Task 1: C1 — Protect static voxels during inflate buffer clear

**Files:**
- Modify: `src/mm_remani_planner/plan_env/src/grid_map.cpp:735`

- [ ] **Step 1: Add `static_occupancy_` guard before clearing `occupancy_buffer_inflate_`**

In `clearAndInflateLocalMap()`, before `md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;` at line 735, insert the guard:

```cpp
// clear outdated data
for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
    for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
      for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z)
      {
        if (md_.static_occupancy_[toAddress(x, y, z)]) continue;
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }
```

- [ ] **Step 2: Build to verify compilation**

```bash
cd /home/hai/E_pan/remani_WS && catkin build remani_planner --no-deps
```

Expected: Build succeeds with no errors.

- [ ] **Step 3: Commit**

```bash
git add src/mm_remani_planner/plan_env/src/grid_map.cpp
git commit -m "fix(grid_map): protect static occupancy during inflate buffer clear

clearAndInflateLocalMap unconditionally cleared occupancy_buffer_inflate_
for all voxels in local bounds, including static map voxels. Static voxels
were never re-inflated because their occupancy_buffer_ remained at the
initial unknown value. Added static_occupancy_ guard to skip them."
```

---

### Task 2: C2 — Protect static voxels during margin clear

**Files:**
- Modify: `src/mm_remani_planner/plan_env/src/grid_map.cpp:660-663,668-670,680-682,687-689,699-701,706-708`

- [ ] **Step 1: Add `static_occupancy_` guard in all six margin-clearing blocks**

Each of the 6 blocks follows the same pattern. Insert `if (md_.static_occupancy_[idx]) continue;` after `int idx = toAddress(x, y, z);` in each:

**Block 1** (z below min_cut, lines 660-663):
```cpp
int idx = toAddress(x, y, z);
if (md_.static_occupancy_[idx]) continue;
md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
md_.distance_buffer_all_[idx] = 10000;
```

**Block 2** (z above max_cut, lines 668-670):
```cpp
int idx = toAddress(x, y, z);
if (md_.static_occupancy_[idx]) continue;
md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
md_.distance_buffer_all_[idx] = 10000;
```

**Block 3** (y below min_cut, lines 680-682):
```cpp
int idx = toAddress(x, y, z);
if (md_.static_occupancy_[idx]) continue;
md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
md_.distance_buffer_all_[idx] = 10000;
```

**Block 4** (y above max_cut, lines 687-689):
```cpp
int idx = toAddress(x, y, z);
if (md_.static_occupancy_[idx]) continue;
md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
md_.distance_buffer_all_[idx] = 10000;
```

**Block 5** (x below min_cut, lines 699-701):
```cpp
int idx = toAddress(x, y, z);
if (md_.static_occupancy_[idx]) continue;
md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
md_.distance_buffer_all_[idx] = 10000;
```

**Block 6** (x above max_cut, lines 706-708):
```cpp
int idx = toAddress(x, y, z);
if (md_.static_occupancy_[idx]) continue;
md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
md_.distance_buffer_all_[idx] = 10000;
```

- [ ] **Step 2: Build to verify compilation**

```bash
cd /home/hai/E_pan/remani_WS && catkin build remani_planner --no-deps
```

Expected: Build succeeds with no errors.

- [ ] **Step 3: Commit**

```bash
git add src/mm_remani_planner/plan_env/src/grid_map.cpp
git commit -m "fix(grid_map): protect static occupancy during margin clear

The six margin-clearing blocks reset occupancy_buffer_ and
distance_buffer_all_ for voxels just outside sensor range without
checking static_occupancy_. This cleared static map ESDF data.
Added static_occupancy_ guard in all six blocks."
```

---

### Task 3: C3 — Set occupancy for all inflated static voxels on map load

**Files:**
- Modify: `src/mm_remani_planner/plan_env/src/grid_map.cpp:994-996`

- [ ] **Step 1: Remove conditional, set `occupancy_buffer_` for every inflated static voxel**

In `cloudCallback()`, global map branch, remove the `if (x == 0 && y == 0 && z == 0)` guard:

```cpp
int idx_inf = toAddress(inf_pt);
md_.occupancy_buffer_inflate_[idx_inf] = 1;
md_.static_occupancy_[idx_inf] = 1;
md_.occupancy_buffer_[idx_inf] = mp_.clamp_max_log_;
```

- [ ] **Step 2: Build to verify compilation**

```bash
cd /home/hai/E_pan/remani_WS && catkin build remani_planner --no-deps
```

Expected: Build succeeds with no errors.

- [ ] **Step 3: Commit**

```bash
git add src/mm_remani_planner/plan_env/src/grid_map.cpp
git commit -m "fix(grid_map): set occupancy for all inflated static map voxels

Static map loading only set occupancy_buffer_ for center voxels of the
inflation kernel (iff x==y==z==0). The 26 surrounding inflated voxels
kept the initial unknown value, causing them to vanish when
clearAndInflateLocalMap re-inflated from occupancy_buffer_ after
dynamic LiDAR updates. Now all inflated static voxels are correctly
initialized."
```

---

### Task 4: Full build and smoke test

- [ ] **Step 1: Full workspace build**

```bash
cd /home/hai/E_pan/remani_WS && catkin build
```

Expected: Full workspace builds with no errors.

- [ ] **Step 2: Launch simulation without extra obstacles (regression test)**

```bash
roslaunch mm_remani exp0_gazebo.launch
```

Expected: Path planning avoids pre-built map as before. RViz occupancy grid shows complete map without holes.

- [ ] **Step 3: Launch simulation with extra obstacles (fix verification)**

In a separate terminal after Gazebo starts:
```bash
# Spawn test obstacles via Gazebo GUI or command line
```

Expected: Path planning avoids BOTH the pre-built map AND the new obstacles. Path does NOT pass through point cloud map.
