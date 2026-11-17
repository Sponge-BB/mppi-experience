#!/usr/bin/python3
"""Read-only inventory, dependency/hash checks and optional ROS graph query."""
import argparse
import glob
import hashlib
import importlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from urllib.parse import urlparse
import xmlrpc.client

ROOT=Path(__file__).resolve().parents[1]
class TimeoutTransport(xmlrpc.client.Transport):
    def make_connection(self,host):
        connection=super().make_connection(host); connection.timeout=2.; return connection


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--master-uri')
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    deps={}
    for name in ['numpy','scipy','yaml','rospy','tf2_ros','sensor_msgs.msg','nav_msgs.msg',
                 'visualization_msgs.msg','active_arm_model','active_arm_mppi','standby_whole_body_mppi']:
        try:
            module=importlib.import_module(name)
            deps[name]=dict(ok=True,path=getattr(module,'__file__',None),version=getattr(module,'__version__',None))
        except Exception as exc:
            deps[name]=dict(ok=False,error=repr(exc))
    manifest_path=ROOT/'config/release_manifest.json'
    manifest=json.loads(manifest_path.read_text()) if manifest_path.exists() else {}
    hashes={}
    for relative,expected in manifest.get('files',{}).items():
        path=ROOT/relative
        actual=hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None
        hashes[relative]=dict(ok=actual==expected,sha256=actual,expected=expected)
    interfaces=[]
    for path in sorted(Path('/sys/class/net').glob('*')):
        try:
            kind=(path/'type').read_text().strip()
            if kind=='280' or path.name.startswith('can'):
                interfaces.append(dict(name=path.name,type=kind,operstate=(path/'operstate').read_text().strip()))
        except OSError:
            pass
    serial=[]
    for pattern in ['/dev/ttyUSB*','/dev/ttyACM*','/dev/serial/by-id/*','/dev/ttyUSB_base']:
        for name in glob.glob(pattern):
            path=Path(name)
            serial.append(dict(path=name,target=str(path.resolve()),
                               readable=os.access(path,os.R_OK),writable=os.access(path,os.W_OK)))
    graph=dict(queried=False)
    if args.master_uri:
        uri=urlparse(args.master_uri)
        if uri.scheme!='http' or not uri.hostname or not uri.port:
            raise SystemExit('Invalid explicit --master-uri')
        try:
            master=xmlrpc.client.ServerProxy(args.master_uri,transport=TimeoutTransport())
            state=master.getSystemState('/standby_preflight')
            topics=master.getTopicTypes('/standby_preflight')
            use_sim_time=master.getParam('/standby_preflight','/use_sim_time')
            graph=dict(queried=True,master_uri=args.master_uri,state=state,topic_types=topics,use_sim_time=use_sim_time)
        except (OSError,xmlrpc.client.Error) as exc:
            graph=dict(queried=True,master_uri=args.master_uri,error=repr(exc))
    report=dict(schema=1,release=str(ROOT),timestamp_unix=time.time(),mode='shadow-only',
        hardware_motion_enabled=False,python=sys.executable,python_no_user_site=os.environ.get('PYTHONNOUSERSITE'),
        frozen_files_pass=bool(hashes) and all(row['ok'] for row in hashes.values()),hashes=hashes,
        dependencies=deps,can_interfaces=interfaces,serial_devices=serial,ros_graph=graph,
        device_inventory_scope='Current process device namespace; a restricted sandbox may hide real devices.',
        cautions=['No CAN or UART device was opened; no driver or actuator was launched.',
                  'Native DH/URDF alignment and actual actuator timing remain unvalidated.'])
    destination=args.output or ROOT/'logs'/('preflight_'+time.strftime('%Y%m%d_%H%M%S',time.gmtime())+'.json')
    destination=destination.resolve()
    if ROOT not in destination.parents:
        raise SystemExit('Preflight report must stay inside this release.')
    destination.parent.mkdir(parents=True,exist_ok=True)
    destination.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(report=str(destination),frozen_files_pass=report['frozen_files_pass'],
                         dependencies_pass=all(row['ok'] for row in deps.values()),
                         can_interfaces=interfaces,serial_devices=serial,ros_graph=graph),ensure_ascii=False,indent=2))


if __name__=='__main__':
    main()
