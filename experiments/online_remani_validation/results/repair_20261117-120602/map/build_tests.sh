#!/usr/bin/env bash
set -euo pipefail
case_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
variant="${1:?Usage: build_tests.sh baseline|axis_layout|negative_subset|combined|float32_scratch}"
case "$variant" in baseline|axis_layout|negative_subset|combined|float32_scratch) ;; *) exit 2 ;; esac
source_dir="$case_dir/$variant"
test_source="$case_dir/test_esdf_worker.cpp"
if [[ "$variant" = negative_subset || "$variant" = combined ]]; then test_source="$case_dir/test_negative_subset.cpp"; fi
if [[ "$variant" = float32_scratch ]]; then test_source="$case_dir/test_float32_scratch.cpp"; fi
test ! -e "$case_dir/build/$variant-worker-tests"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MALLOC_ARENA_MAX=2
ulimit -v 614400
sha256sum "$source_dir/esdf_worker.cpp" "$source_dir/include/plan_env/esdf_worker.h" "$test_source" > "$case_dir/build/$variant.tests.inputs.before.sha256"
extra=()
if [[ "$variant" = negative_subset || "$variant" = combined ]]; then extra=(-DBENCH_NEGATIVE_SUBSET_METRICS=1); fi
/usr/bin/python3 "$case_dir/record_compile_resources.py" "$case_dir/build/$variant.tests.compiler_resources.json" nice -n 15 /usr/bin/g++ -std=c++14 -O2 -DNDEBUG -g0 -Wall -Wextra --param=ggc-min-expand=5 --param=ggc-min-heapsize=16384 "${extra[@]}" -I"$source_dir/include" -I/usr/include/eigen3 -I/opt/ros/noetic/include "$source_dir/esdf_worker.cpp" "$test_source" -L/opt/ros/noetic/lib -Wl,-rpath,/opt/ros/noetic/lib -lrostime -lgtest -lgtest_main -pthread -o "$case_dir/build/$variant-worker-tests" > "$case_dir/build/$variant.tests.build.log" 2> "$case_dir/build/$variant.tests.resources.log"
sha256sum "$source_dir/esdf_worker.cpp" "$source_dir/include/plan_env/esdf_worker.h" "$test_source" > "$case_dir/build/$variant.tests.inputs.after.sha256"
cmp "$case_dir/build/$variant.tests.inputs.before.sha256" "$case_dir/build/$variant.tests.inputs.after.sha256"
sha256sum "$case_dir/build/$variant-worker-tests" > "$case_dir/build/$variant.tests.binary.sha256"
