Prepared offline dense-worker validation; compilation/runs are root-agent scheduled.

Files in this directory are experiment-only. No runtime source, deployed library,
ROS master, robot process, or hardware settings are modified by these tools.

Input: 16 independent valid/coherent diagnostic crops from the 11 MB
dynamic_geometry_capture_20261114-175950/occupancy_evidence.bag. Each is 82x82x60
at 0.05m. The world ROI moves. Each crop is independently rebased to grid zero;
changed_cells is ALWAYS empty. No cross-ROI change inference or incremental
performance claim is made. Worker input is INFLATED occupancy (bit1), not raw bit0.
Do not compare these closed-crop outputs with global ESDF values at crop borders.

Synthetic supplemental inputs: 200x184x60=2,208,000 cells, nonzero grid offset,
all-free / seeded mixed occupancy / thick solids with a cavity and map boundary.
These test scale and signed negative-distance behavior, NOT a recorded real job.

Native distance_cap=0, no clipping or quantization. Every output double must be
finite and old/new equal at every voxel; mismatches and examples remain in the
summary and cause nonzero exit. First repetition saves the full uncapped field;
all repetitions log phase times, queue/compute/submit-wait, fallback count and RSS.
Timing excludes file IO, allocation of input job, fusion, GridMap commit/export,
ROS communication, and MPPI. It is not end-to-end real-robot latency.

Before source: rollback/20261114-192539-before-dense-edt/files/.../plan_env.
After source: current src/mm_remani_planner/plan_env. Each binary compiles worker
directly and never links/swaps deployed libplan_env. Sources/harness/binary hashes
are bound before and after compilation and checked before/after running.

Commands (run from this case directory, ONE COMMAND AT A TIME):
  source /opt/ros/noetic/setup.bash
  PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /usr/bin/python3 export_dense_fixtures.py
  bash build_dense_benchmark.sh before
  bash build_dense_benchmark.sh after
  PYTHONNOUSERSITE=1 /usr/bin/python3 run_dense_benchmark.py --threads 3 --repeats 3 --label final

The export refuses an existing fixtures directory. Build refuses an existing
variant binary. Each run requires a new label, preserving previous evidence.
Use --threads 1 --label serial for true one-compute-thread diagnostics if needed;
OMP_NUM_THREADS alone does not set the worker's explicit C++ thread pool.
Do not run variants/tests/compilation concurrently. No compilation or benchmark
is automatically invoked by export or script preparation.

Archive the four source scripts/C++ and this README plus small manifests and
summaries/logs. fixtures/, build/, and runs_*/{before,after}/ are regenerable data;
exclude these directories from the compact version archive. Occupancy fixtures
total about 13 MB, one before/after raw-output pair about 210 MB. No bag is copied.
