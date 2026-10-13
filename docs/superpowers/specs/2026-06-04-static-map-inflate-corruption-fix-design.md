# Static Map Inflate Corruption Fix

## Date

2026-06-04

## Problem

In Gazebo simulation with `exp0_gazebo.launch`, when extra obstacles are manually added to the Gazebo world, the planned path frequently passes through the pre-built point cloud map. Without extra obstacles, path planning correctly avoids the map.

### Root Cause

Three related bugs in `grid_map.cpp` cause static (pre-loaded map) occupancy data to be silently destroyed when LiDAR observations trigger the occupancy update pipeline.

**Trigger condition**: Without extra obstacles in Gazebo, the LiDAR only sees the ground plane. Ground points are filtered out (`pt_world.z < ground_height + 0.15`), resulting in `proj_points_cnt == 0`. The occupancy update pipeline never runs, so the static map stays intact.

When extra obstacles are spawned, the LiDAR sees them above ground, `cloudOdomCallback` populates projection points, and the full update cycle executes — destroying static map data.

#### Bug 1 (PRIMARY): `clearAndInflateLocalMap` unconditionally clears `occupancy_buffer_inflate_` for all voxels in local bounds

**File**: `src/mm_remani_planner/plan_env/src/grid_map.cpp`, lines 731-736

The function clears `occupancy_buffer_inflate_` (the buffer used by ESDF and collision checking) for all voxels within `local_bound_min_` to `local_bound_max_`, including static map voxels. Re-inflation happens only from `occupancy_buffer_` values above the occupancy threshold. Static map inflated voxels (non-center positions from the inflation kernel) have their `occupancy_buffer_` at the initial "unknown" value, far below threshold — so they are never re-inflated, creating holes in the map.

#### Bug 2 (SECONDARY): Margin clearing does not respect `static_occupancy_`

**File**: `src/mm_remani_planner/plan_env/src/grid_map.cpp`, lines 656-710

The six margin-clearing blocks reset `occupancy_buffer_` and `distance_buffer_all_` for voxels just outside the sensor range without checking `static_occupancy_`. This can clear static map ESDF data in a band around the robot's current sensor footprint.

#### Bug 3 (DATA): Static map loading only sets `occupancy_buffer_` for center voxels of inflation kernel

**File**: `src/mm_remani_planner/plan_env/src/grid_map.cpp`, lines 994-996

When loading the pre-built map via `cloudCallback` (global map mode), only the center voxel of each inflated point gets `occupancy_buffer_` set to `clamp_max_log_`. The 26 surrounding inflated voxels have `static_occupancy_ = 1` but `occupancy_buffer_` remains at the initial value. This creates an internal data inconsistency that Bugs 1 and 2 exploit.

## Fix (Plan C)

Three modifications in `src/mm_remani_planner/plan_env/src/grid_map.cpp`:

### C1: Protect static voxels during inflate clear

**Lines 731-736**: Add `static_occupancy_` guard before clearing `occupancy_buffer_inflate_`:

```cpp
for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
  for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
    for (int z = md_.local_bound_min_(2); z <= md_.local_bound_max_(2); ++z)
    {
      if (md_.static_occupancy_[toAddress(x, y, z)]) continue;
      md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
    }
```

### C2: Protect static voxels during margin clear

**Lines 656-710**: Add `static_occupancy_` guard in all six margin-clearing blocks before resetting `occupancy_buffer_` and `distance_buffer_all_`:

```cpp
int idx = toAddress(x, y, z);
if (md_.static_occupancy_[idx]) continue;
md_.occupancy_buffer_[idx] = mp_.clamp_min_log_ - mp_.unknown_flag_;
md_.distance_buffer_all_[idx] = 10000;
```

### C3: Set `occupancy_buffer_` for all inflated static voxels

**Lines 994-996**: Remove conditional, set `occupancy_buffer_` for every inflated static voxel:

```cpp
md_.occupancy_buffer_inflate_[idx_inf] = 1;
md_.static_occupancy_[idx_inf] = 1;
md_.occupancy_buffer_[idx_inf] = mp_.clamp_max_log_;
```

## Scope

- File modified: `src/mm_remani_planner/plan_env/src/grid_map.cpp`
- ~10 lines changed
- No API changes, no new dependencies
- No configuration changes needed

## Testing

- Launch `exp0_gazebo.launch`, verify path avoids pre-built map (regression)
- Launch with manually added Gazebo obstacles, verify path avoids both pre-built map AND new obstacles
- Verify path still reaches waypoints successfully
- Monitor RViz occupancy grid visualization for unexpected holes
