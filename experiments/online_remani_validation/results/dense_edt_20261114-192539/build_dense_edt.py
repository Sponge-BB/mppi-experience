import json,os,signal,subprocess,time,sys
from pathlib import Path
x=json.loads(Path('/tmp/remani_dense_edt_version.json').read_text());case=Path(x['case']);label=sys.argv[1] if len(sys.argv)>1 else 'build';log=(case/(label+'.log')).open('w')
cmd=['nice','-n','15','cmake','--build',x['build'],'--parallel','1','--target']+(sys.argv[2:] if len(sys.argv)>2 else ['plan_env','test_esdf_worker','test_grid_map_parallel','test_dynamic_point_buffer'])
p=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);beg=time.monotonic();peak=0;min_avail=1e30;reason=None
while p.poll() is None:
 rss=0
 for q in Path('/proc').glob('[0-9]*/stat'):
  try:
   f=q.read_text().rpartition(')')[2].split()
   if int(f[2])==p.pid:rss+=max(0,int(f[21]))*os.sysconf('SC_PAGE_SIZE')
  except (OSError,ValueError,IndexError):pass
 avail=next(int(s.split()[1])*1024 for s in Path('/proc/meminfo').read_text().splitlines() if s.startswith('MemAvailable:'));peak=max(peak,rss);min_avail=min(min_avail,avail)
 if avail<1500*1024**2:reason='available_RAM_below_1500MiB'
 if rss>1800*1024**2:reason='own_build_RSS_above_1800MiB'
 if time.monotonic()-beg>900:reason='own_build_time_limit_900s'
 if reason:
  os.killpg(p.pid,signal.SIGTERM)
  try:p.wait(timeout=5)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
  break
 time.sleep(.5)
log.close();r={'command':cmd,'exit_code':p.wait(),'elapsed_s':time.monotonic()-beg,'peak_build_group_rss_mib':peak/1024**2,'lowest_available_mib':min_avail/1024**2,'stop_reason':reason,'jobs':1,'clean_build':True,'build_dir':x['build'],'runtime_destination_untouched':True};(case/(label+'_result.json')).write_text(json.dumps(r,indent=2)+'\n');print(json.dumps(r,indent=2),flush=True)
raise SystemExit(0 if r['exit_code']==0 and not reason else 1)
