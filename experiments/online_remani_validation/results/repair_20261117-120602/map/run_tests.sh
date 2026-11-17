#!/usr/bin/env bash
set -euo pipefail
case_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
variant="${1:?variant}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MALLOC_ARENA_MAX=2
ulimit -v 614400
/usr/bin/python3 "$case_dir/record_compile_resources.py" "$case_dir/build/$variant.tests.run_resources.json" nice -n 10 "$case_dir/build/$variant-worker-tests" "--gtest_output=xml:$case_dir/build/$variant.tests.xml" > "$case_dir/build/$variant.tests.run.log" 2>&1
