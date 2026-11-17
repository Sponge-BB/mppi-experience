from pathlib import Path
import json,os,subprocess,time
p=Path("/home/l/remani_ws/experiments/online_remani_validation/results/active_arm_20261104_01/shutdown_receipt.json")
time.sleep(4.)
r=json.loads(p.read_text());env=os.environ.copy();env["DBUS_SESSION_BUS_ADDRESS"]="unix:path=/run/user/1000/bus"
command=["/usr/bin/gnome-session-quit","--power-off","--no-prompt"]
try:
 done=subprocess.run(command,env=env,capture_output=True,text=True,timeout=20)
 accepted=done.returncode==0 and "Failed to call" not in done.stderr and "WARNING" not in done.stderr
 result=dict(method="normal desktop power-off after 4s tool-release delay",command=command,returncode=done.returncode,stdout=done.stdout.strip(),stderr=done.stderr.strip(),request_accepted=accepted)
except Exception as exc:result=dict(method="delayed normal desktop power-off",error=str(exc),request_accepted=False)
r["attempts"].append(result);r.update(result);p.write_text(json.dumps(r,ensure_ascii=False,indent=2)+"\n");os.sync()
print(json.dumps(result,ensure_ascii=False),flush=True)
