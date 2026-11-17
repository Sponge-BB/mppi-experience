#!/usr/bin/env bash
set -euo pipefail
variant="${1:?Usage: bash build_dense_benchmark.sh before|after}"
case_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace=/home/l/remani_ws
case "$variant" in
  before) source_dir="$workspace/rollback/20261114-192539-before-dense-edt/files/src/mm_remani_planner/plan_env" ;;
  after) source_dir="$workspace/src/mm_remani_planner/plan_env" ;;
  *) echo 'Choose before or after' >&2; exit 2 ;;
esac
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MALLOC_ARENA_MAX=2
mkdir -p "$case_dir/build"
binary="$case_dir/build/$variant-worker-benchmark"
if [[ -e "$binary" ]]; then
  echo "Refusing to overwrite existing benchmark: $binary" >&2
  exit 2
fi
sha256sum "$source_dir/src/esdf_worker.cpp" "$source_dir/include/plan_env/esdf_worker.h" \
  "$case_dir/dense_worker_benchmark.cpp" > "$case_dir/build/$variant.inputs.before.sha256"
/usr/bin/python3 "$case_dir/timed_compile.py" "$case_dir/build/$variant.compiler_resources.json" \
  nice -n 15 /usr/bin/g++ -std=c++14 -O3 -g0 -Wall -Wextra \
  --param=ggc-min-expand=5 --param=ggc-min-heapsize=16384 \
  "-DBENCH_VARIANT=\"$variant\"" \
  -I"$source_dir/include" -I/usr/include/eigen3 -I/opt/ros/noetic/include \
  "$source_dir/src/esdf_worker.cpp" "$case_dir/dense_worker_benchmark.cpp" \
  -L/opt/ros/noetic/lib -Wl,-rpath,/opt/ros/noetic/lib -lrostime -pthread \
  -o "$binary" > "$case_dir/build/$variant.build.log" 2>&1
sha256sum "$source_dir/src/esdf_worker.cpp" "$source_dir/include/plan_env/esdf_worker.h" \
  "$case_dir/dense_worker_benchmark.cpp" > "$case_dir/build/$variant.inputs.after.sha256"
cmp "$case_dir/build/$variant.inputs.before.sha256" "$case_dir/build/$variant.inputs.after.sha256"
sha256sum "$binary" > "$case_dir/build/$variant.executable.sha256"
echo "Built $variant only; no benchmark or robot node started."
