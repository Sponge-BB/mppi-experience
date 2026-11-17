# Tracker diagnostics candidate — staged only

Apply scope: exactly the four relative source files in candidate_manifest.json. Production src/devel were not modified by this task. candidate.patch is a unified patch against the original_manifest.json hashes. Root must save the rollback baseline and verify the four base hashes before applying.

Runtime change: no motion threshold, tracked geometry, association, qualifier/exit condition, class layout or TrackedDetection layout changes. The old two-argument update symbol remains. The optional overload captures existing scalar evidence and the original one-time surface check result. The original one-Hz DynamicTracker summary adds at most 12 cluster records, an explicit truncation count, source time, human-readable failing scalar predicates and surface passed/rejected/not_evaluated. A missing surface check is not printed as a rejected check. No new message, topic, timer, debug rate or geometry query.

Validation (offline only):
- python3 /tmp/remani_tracker_diag_stage/validate_standalone.py
  Builds only Eigen core+gtest with static system gtest, serially, process-group 512MiB cap and 1536MiB available-memory floor. All 37 tests passed: previous 32 plus five production-profile diagnostics fixtures. XML and exact compile commands/RSS are in standalone_build/tests.xml and standalone_validation.json.
- python3 /tmp/remani_tracker_diag_stage/validate_original_equivalence.py
  Builds the frozen original core and a single driver against the original header, then links the same driver against original and candidate core. Six deterministic sequences (walking/stop/missed frame + second object; stationary sliding view; background anchor release; sparse moving cluster; observation gaps; resets), 546 update calls plus size header. Output serializes each double by exact uint64 bits and compares all returned fields and persistent counters/class sizes. All 547 lines/61418 bytes equal. Hashes, commands and RSS are in original_comparison_validation.json. This covers the fixture set, not every possible input.
- profile_provenance.json verifies every frozen fixture core parameter against exp0_real.launch plus the node's voxel-derived surface tolerance, without retuning.

Not validated: ROS/PCL tracker node was deliberately not compiled here; actual one-Hz log publication and real person classification require root's normal bounded build and later observation. Existing geometry bag is not the exact tracker input, so these fixtures do not prove why the recorded human failed classification. Do not claim dynamic-recognition behavior improved from this diagnostic-only change.

Deployment targets after approved baseline: dynamic_obstacle_tracker_core library and dynamic_obstacle_tracker_node executable; include the gtest executable if rebuilt. Source/deployment must be saved in the same rollback version. No external SDK or hardware state was changed.
