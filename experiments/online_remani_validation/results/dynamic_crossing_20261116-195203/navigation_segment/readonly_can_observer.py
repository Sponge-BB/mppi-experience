import collections
import datetime
import json
from pathlib import Path
import signal
import socket
import struct
import time

out=Path('/home/l/remani_ws/experiments/online_remani_validation/results/dynamic_crossing_20261116-195203/navigation_segment')
running=True
def stop(*_):
    global running
    running=False
for n in (signal.SIGINT,signal.SIGTERM):signal.signal(n,stop)
s=socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW)
ids=[0x150,0x151,0x155,0x156,0x157,0x2A1,0x2A5,0x2A6,0x2A7,*range(0x261,0x267)]
# CAN_ERR_FLAG is CAN_INV_FILTER in the filter API. Do not put it in the mask.
# Default SocketCAN filter: keep every data frame, including startup/enable IDs.
s.setsockopt(socket.SOL_SOCKET,35,1)
s.setsockopt(socket.SOL_SOCKET,40,1)
s.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,262144)
s.settimeout(.1)
s.bind(('can1',))
start=time.monotonic();next_status=start;count=0;drops=0;counts=collections.Counter();maxlag=0
meta={'start_wall':time.time(),'record_bytes':40,'format':'=QQQIB3x8s',
      'file':'can_frames_fixed.bin','interface':'can1','filter':'all CAN data frames; SocketCAN default filter',
      'can_tx_calls':0,'limit':'600s / 64MiB',
      'caveat':'Passive companion for repair acceptance test; kernel receive time is not motor transmit time.'}
(out/'can_fixed_metadata.json').write_text(json.dumps(meta,indent=2)+'\n')
print('CAN_READY '+json.dumps(meta),flush=True)
try:
 with (out/'can_frames_fixed.bin').open('xb',buffering=262144) as f:
  while running and time.monotonic()-start<600 and count*40<64*1024**2:
   try:frame,anc,flags,_=s.recvmsg(16,128)
   except socket.timeout:continue
   wall=time.time_ns();steady=time.monotonic_ns();kernel=None
   for level,kind,payload in anc:
    if level==socket.SOL_SOCKET and kind==35:
     sec,ns=struct.unpack_from('@ll',payload);kernel=sec*1000000000+ns
    elif level==socket.SOL_SOCKET and kind==40:drops=struct.unpack_from('=I',payload)[0]
   if kernel is None or len(frame)!=16 or flags&socket.MSG_CTRUNC:raise RuntimeError('incomplete timestamp/frame')
   i,dlc,data=struct.unpack('=IB3x8s',frame)
   f.write(struct.pack('=QQQIB3x8s',kernel,wall,steady,i,dlc,data));count+=1;counts[hex(i)]+=1
   maxlag=max(maxlag,(wall-kernel)/1e6)
   if time.monotonic()>=next_status:
    f.flush();meta.update(running=True,count=count,drops=drops,counts=dict(counts),max_reader_lag_ms=maxlag,last_wall=time.time())
    (out/'can_fixed_status.json').write_text(json.dumps(meta,indent=2)+'\n');next_status=time.monotonic()+1
finally:
 s.close();meta.update(running=False,count=count,drops=drops,counts=dict(counts),max_reader_lag_ms=maxlag,end_wall=time.time())
 (out/'can_fixed_summary.json').write_text(json.dumps(meta,indent=2)+'\n')
 print('CAN_SAVED '+str(out),flush=True)
