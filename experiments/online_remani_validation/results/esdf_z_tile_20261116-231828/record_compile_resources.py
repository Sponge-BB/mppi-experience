#!/usr/bin/env python3
import json,resource,subprocess,sys,time
from pathlib import Path
start=time.monotonic()
result=subprocess.run(sys.argv[2:])
Path(sys.argv[1]).write_text(json.dumps({"command":sys.argv[2:],"exit_code":result.returncode,"elapsed_s":time.monotonic()-start,"peak_child_rss_MiB":resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss/1024.},indent=2)+"\n")
raise SystemExit(result.returncode)
