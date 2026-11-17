#!/usr/bin/env bash
set -euo pipefail
case_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
variant="${1:?Usage: bash build_pair.sh baseline|z_tile8}"
case "$variant" in baseline|z_tile8) ;; *) exit 2 ;; esac
source_dir="$case_dir/$variant"
mkdir -p "$case_dir/build"
if [[ -e "$case_dir/build/$variant-worker-benchmark" ]]; then
  echo 'Refusing to overwrite an existing binary' >&2; exit 2
fi
available_kib="$(awk '/^MemAvailable:/ {print $2}' /proc/meminfo)"
if (( available_kib < 1536000 )); then
  echo 'Refusing compilation: less than1500MiB MemAvailable' >&2; exit 3
fi
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MALLOC_ARENA_MAX=2
ulimit -v 614400
sha256sum "$source_dir/esdf_worker.cpp" "$source_dir/include/plan_env/esdf_worker.h" \
  "$case_dir/dense_worker_benchmark.cpp" > "$case_dir/build/$variant.inputs.before.sha256"
/usr/bin/python3 "$case_dir/record_compile_resources.py" "$case_dir/build/$variant.compiler_resources.json" nice -n 15 /usr/bin/g++ -std=c++14 -O3 -DNDEBUG -g0 -Wall -Wextra \
  --param=ggc-min-expand=5 --param=ggc-min-heapsize=16384 \
  "-DBENCH_VARIANT=\"$variant\"" \
  -I"$source_dir/include" -I/usr/include/eigen3 -I/opt/ros/noetic/include \
  "$source_dir/esdf_worker.cpp" "$case_dir/dense_worker_benchmark.cpp" \
  -L/opt/ros/noetic/lib -Wl,-rpath,/opt/ros/noetic/lib -lrostime -pthread \
  -o "$case_dir/build/$variant-worker-benchmark" \
  > "$case_dir/build/$variant.build.log" 2> "$case_dir/build/$variant.resources.log"
sha256sum "$source_dir/esdf_worker.cpp" "$source_dir/include/plan_env/esdf_worker.h" \
  "$case_dir/dense_worker_benchmark.cpp" > "$case_dir/build/$variant.inputs.after.sha256"
cmp "$case_dir/build/$variant.inputs.before.sha256" "$case_dir/build/$variant.inputs.after.sha256"
sha256sum "$case_dir/build/$variant-worker-benchmark" > "$case_dir/build/$variant.binary.sha256"
echo "Built $variant in experiment directory only."
