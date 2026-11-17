#!/usr/bin/env bash
set -euo pipefail
case_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
variant="${1:?Usage: build_candidate.sh baseline|axis_layout|negative_subset|combined|float32_scratch}"
case "$variant" in baseline|axis_layout|negative_subset|combined|float32_scratch) ;; *) exit 2 ;; esac
source_dir="$case_dir/$variant"
bench_source="$case_dir/dense_worker_benchmark.cpp"
if [[ "$variant" = float32_scratch ]]; then bench_source="$case_dir/dense_worker_benchmark_float.cpp"; fi
test ! -e "$case_dir/build/$variant-worker-benchmark"
available_kib="$(awk '/^MemAvailable:/ {print $2}' /proc/meminfo)"
if (( available_kib < 1536000 )); then echo 'Less than 1500 MiB available; compile deferred' >&2; exit 3; fi
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MALLOC_ARENA_MAX=2
ulimit -v 614400
sha256sum "$source_dir/esdf_worker.cpp" "$source_dir/include/plan_env/esdf_worker.h" "$bench_source" > "$case_dir/build/$variant.inputs.before.sha256"
extra=()
if [[ "$variant" = negative_subset || "$variant" = combined ]]; then extra=(-DBENCH_NEGATIVE_SUBSET_METRICS=1); fi
/usr/bin/python3 "$case_dir/record_compile_resources.py" "$case_dir/build/$variant.compiler_resources.json" nice -n 15 /usr/bin/g++ -std=c++14 -O3 -DNDEBUG -g0 -Wall -Wextra \
  --param=ggc-min-expand=5 --param=ggc-min-heapsize=16384 "-DBENCH_VARIANT=\"$variant\"" "${extra[@]}" \
  -I"$source_dir/include" -I/usr/include/eigen3 -I/opt/ros/noetic/include \
  "$source_dir/esdf_worker.cpp" "$bench_source" \
  -L/opt/ros/noetic/lib -Wl,-rpath,/opt/ros/noetic/lib -lrostime -pthread \
  -o "$case_dir/build/$variant-worker-benchmark" > "$case_dir/build/$variant.build.log" 2> "$case_dir/build/$variant.resources.log"
sha256sum "$source_dir/esdf_worker.cpp" "$source_dir/include/plan_env/esdf_worker.h" "$bench_source" > "$case_dir/build/$variant.inputs.after.sha256"
cmp "$case_dir/build/$variant.inputs.before.sha256" "$case_dir/build/$variant.inputs.after.sha256"
sha256sum "$case_dir/build/$variant-worker-benchmark" > "$case_dir/build/$variant.binary.sha256"
