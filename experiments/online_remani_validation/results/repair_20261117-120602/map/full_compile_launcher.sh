#!/bin/sh
exec "$@" -g0 --param=ggc-min-expand=5 --param=ggc-min-heapsize=16384
