#!/usr/bin/python3
"""Freeze a separate, read-only-to-hardware standby MPPI release.

This script never launches ROS, a device driver, a controller, or installs a
dependency. Existing frozen files may only be reproduced byte-for-byte.
"""
import argparse
import hashlib
import json
from pathlib import Path
import stat
import xml.etree.ElementTree as ET


HERE = Path(__file__).resolve().parent
WORKSPACE = HERE.parents[1]
RELEASE = HERE / 'releases' / 'standby_20261105_01'
SOURCE = WORKSPACE / 'experiments/online_remani_validation'
CASE = SOURCE / 'results/standby_arm_influence_20261105_01/current_standby_extension_valid'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path, value, executable=False):
    data = value.encode() if isinstance(value, str) else value
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and path.read_bytes() != data:
        raise RuntimeError('Refusing to overwrite an existing different frozen file: '+str(path))
    path.write_bytes(data)
    if executable:
        path.chmod(path.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP)


def json_write(path, data):
    write(path, json.dumps(data, ensure_ascii=False, indent=2)+'\n')


def freeze_model():
    source = WORKSPACE/'src/my_description/robot/mm.urdf'
    root = ET.parse(source).getroot()
    # The core reads kinematics and joint limits only. Render authoritative
    # spheres as markers instead of depending on uncalibrated mesh packages.
    visual_removed = 0
    for link in root.findall('link'):
        for tag in ['visual', 'collision']:
            for element in list(link.findall(tag)):
                link.remove(element); visual_removed += 1
    removed = len(root.findall('gazebo'))
    for element in list(root.findall('gazebo')):
        root.remove(element)
    joint6 = root.find("joint[@name='joint6']/limit")
    if joint6 is None:
        raise RuntimeError('Missing joint6 limit in source URDF')
    joint6.set('lower', '-0.35'); joint6.set('upper', '0.35')
    write(RELEASE/'model/mm.urdf', ET.tostring(root, encoding='utf-8', xml_declaration=True)+b'\n')
    json_write(RELEASE/'model/provenance.json', {
        'source_urdf': str(source.relative_to(WORKSPACE)), 'source_urdf_sha256': digest(source),
        'transformation': 'self-contained kinematic URDF: visual/collision mesh elements and unused Gazebo elements removed; joint6 narrowed to [-0.35,0.35]',
        'gazebo_elements_removed': removed, 'visual_collision_elements_removed': visual_removed,
        'joint_limit_overrides': {'joint6': [-.35, .35]},
        'geometry_scope': 'Native DH spheres; visual URDF agreement has not been calibrated on hardware.'})


ENV = r'''#!/usr/bin/env bash
set -euo pipefail
STANDBY_RELEASE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
export STANDBY_RELEASE_DIR
unset ROS_PACKAGE_PATH CMAKE_PREFIX_PATH PYTHONPATH ROSLISP_PACKAGE_DIRECTORIES ROS_DISTRO ROS_ROOT ROS_ETC_DIR
unset PYTHONHOME PYTHONUSERBASE VIRTUAL_ENV CONDA_PREFIX LD_LIBRARY_PATH
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
set +u
source /opt/ros/noetic/setup.bash
set -u
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export PYTHONPATH="$STANDBY_RELEASE_DIR/core:$STANDBY_RELEASE_DIR/runtime${PYTHONPATH:+:$PYTHONPATH}"
export ROS_HOME="$STANDBY_RELEASE_DIR/run/ros_home"
export ROS_LOG_DIR="$STANDBY_RELEASE_DIR/logs/ros"
export ROS_NAMESPACE=/standby_trial_20261105_01
unset ROS_IP ROS_HOSTNAME ROS_MASTER_URI
mkdir -p -- "$ROS_HOME" "$ROS_LOG_DIR"
'''


SESSION = r'''#!/usr/bin/python3
"""Own only this release's process groups; never send actuator messages."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
from urllib.parse import urlparse
import xmlrpc.client

ROOT=Path(__file__).resolve().parents[1]
STATE=ROOT/'run/session.json'
NAMESPACE='/standby_trial_20261105_01'


def save(data):
    STATE.parent.mkdir(parents=True,exist_ok=True)
    temp=STATE.with_suffix('.tmp')
    temp.write_text(json.dumps(data,indent=2)+'\n'); temp.replace(STATE)


def ticks(pid):
    try:
        return Path('/proc/'+str(pid)+'/stat').read_text().rsplit(')',1)[1].split()[19]
    except (OSError,IndexError):
        return None


def owned(entry):
    pid=int(entry['pid'])
    try:
        env=Path('/proc/'+str(pid)+'/environ').read_bytes().split(b'\0')
        return ticks(pid)==entry['start_ticks'] and os.getpgid(pid)==pid and \
            ('STANDBY_RELEASE_DIR='+str(ROOT)).encode() in env
    except OSError:
        return False


def launch(command,log,env):
    with log.open('ab',buffering=0) as stream:
        process=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,
                                 cwd=ROOT,env=env,start_new_session=True)
    return dict(pid=process.pid,start_ticks=ticks(process.pid),command=command,log=str(log))


class TimeoutTransport(xmlrpc.client.Transport):
    def make_connection(self,host):
        connection=super().make_connection(host); connection.timeout=1.; return connection


def master_ok(uri):
    try:
        result=xmlrpc.client.ServerProxy(uri,transport=TimeoutTransport()).getPid('/standby_release_check')
        return result[0]==1
    except (OSError,xmlrpc.client.Error):
        return False


def stop(data):
    stopped=[]
    for entry in reversed(data.get('processes',[])):
        if not owned(entry):
            continue
        pid=entry['pid']; os.killpg(pid,signal.SIGINT)
        deadline=time.monotonic()+8.
        while owned(entry) and time.monotonic()<deadline:
            time.sleep(.1)
        if owned(entry):
            os.killpg(pid,signal.SIGTERM)
        stopped.append(pid)
    data.update(status='stopped',stopped_pids=stopped,stopped_wall=time.time()); save(data)
    return stopped


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['start-shadow','start-monitor','stop','status','view'])
    parser.add_argument('--master-uri')
    parser.add_argument('--advertise-host')
    args=parser.parse_args()
    (ROOT/'run').mkdir(exist_ok=True)
    with (ROOT/'run/session.lock').open('a') as lock:
        fcntl.flock(lock,fcntl.LOCK_EX)
        data=json.loads(STATE.read_text()) if STATE.exists() else {}
        if args.action=='stop':
            print(json.dumps(dict(stopped_pids=stop(data),release=str(ROOT)))); return
        live=[entry for entry in data.get('processes',[]) if owned(entry)]
        if args.action=='status':
            print(json.dumps(dict(record=data,live_processes=live),indent=2)); return
        if args.action=='view':
            uri=args.master_uri or data.get('master_uri')
            parsed=urlparse(uri or '')
            if parsed.scheme!='http' or not parsed.hostname or not parsed.port or not master_ok(uri):
                raise SystemExit('The explicit ROS master or this release monitor must be reachable before RViz.')
            rviz=Path('/opt/ros/noetic/bin/rviz')
            if not rviz.is_file():
                raise SystemExit('System ROS RViz is not installed; no dependency was changed.')
            env=os.environ.copy(); env['ROS_MASTER_URI']=uri
            if parsed.hostname in ('127.0.0.1','localhost'):
                env['ROS_IP']='127.0.0.1'; env['ROS_HOSTNAME']='localhost'
            elif args.advertise_host:
                env['ROS_HOSTNAME']=args.advertise_host
            else:
                raise SystemExit('A remote master also requires --advertise-host <this-host-address>.')
            log_dir=Path(data['run_dir']) if data.get('run_dir') else ROOT/'logs'
            log_dir.mkdir(parents=True,exist_ok=True)
            entry=launch([str(rviz),'-d',str(ROOT/'config/shadow.rviz'),
                          '__name:=standby_shadow_view','__ns:='+NAMESPACE],log_dir/'rviz.log',env)
            data.setdefault('processes',[]).append(entry); data['master_uri']=uri; save(data)
            print(json.dumps(entry,indent=2)); return
        if live:
            raise SystemExit('This release already has live processes; inspect bin/status.sh or bin/stop.sh.')
        node=ROOT/'runtime/hardware_shadow_node.py'
        if not node.is_file():
            raise SystemExit('Missing frozen runtime/hardware_shadow_node.py; do not start an incomplete release.')
        monitor=args.action=='start-monitor'
        uri='http://127.0.0.1:11461' if monitor else args.master_uri
        parsed=urlparse(uri or '')
        if parsed.scheme!='http' or not parsed.hostname or not parsed.port or parsed.username or parsed.password:
            raise SystemExit('Provide an explicit HTTP ROS master URI, e.g. --master-uri http://127.0.0.1:11311')
        env=os.environ.copy(); env['ROS_MASTER_URI']=uri
        if parsed.hostname in ('127.0.0.1','localhost'):
            env['ROS_IP']='127.0.0.1'; env['ROS_HOSTNAME']='localhost'
        elif args.advertise_host:
            env['ROS_HOSTNAME']=args.advertise_host
        else:
            raise SystemExit('A remote master also requires --advertise-host <this-host-address>.')
        stamp=time.strftime('%Y%m%d_%H%M%S',time.gmtime())
        run=ROOT/'logs'/stamp; run.mkdir(parents=True,exist_ok=False)
        env['STANDBY_RUN_DIR']=str(run)
        data=dict(release=str(ROOT),namespace=NAMESPACE,mode='shadow',hardware_publishers=False,
                  master_uri=uri,run_dir=str(run),status='starting',processes=[])
        try:
            if monitor:
                with socket.socket() as probe:
                    if probe.connect_ex(('127.0.0.1',11461))==0:
                        raise RuntimeError('Port 11461 is occupied; refusing to reuse or stop another master.')
                entry=launch(['/opt/ros/noetic/bin/roscore','-p','11461'],run/'roscore.log',env)
                data['processes'].append(entry); save(data)
                deadline=time.monotonic()+12.
                while not master_ok(uri) and time.monotonic()<deadline:
                    if not owned(entry):
                        raise RuntimeError('The private roscore exited; inspect '+str(run/'roscore.log'))
                    time.sleep(.15)
            if not master_ok(uri):
                raise RuntimeError('ROS master is not reachable; no shadow node was launched.')
            entry=launch(['/usr/bin/python3',str(node),'--release-dir',str(ROOT),
                          '--namespace',NAMESPACE],run/'shadow.log',env)
            data['processes'].append(entry); save(data)
            time.sleep(.6)
            if not owned(entry):
                raise RuntimeError('Shadow process exited; inspect '+str(run/'shadow.log'))
            data.update(status='running',started_wall=time.time()); save(data)
            print(json.dumps(data,indent=2))
        except BaseException:
            stop(data)
            raise


if __name__=='__main__':
    main()
'''


PREFLIGHT = r'''#!/usr/bin/python3
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
'''


RVIZ = '''Panels:
  - Class: rviz/Displays
    Name: Displays
  - Class: rviz/Views
    Name: Views
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz/Grid
      Name: Ground grid
      Enabled: true
      Cell Size: 0.25
      Plane: XY
      Reference Frame: odom
    - Class: rviz/MarkerArray
      Name: Measured model and shadow suggestions
      Enabled: true
      Marker Topic: /standby_trial_20261105_01/markers
      Queue Size: 1
    - Class: rviz/PointCloud2
      Name: Actual raw cloud (optional)
      Enabled: false
      Topic: /livox/lidar_pcl2
      Queue Size: 1
      Style: Points
      Size (Pixels): 2
      Color Transformer: AxisColor
      Position Transformer: XYZ
    - Class: rviz/PointCloud2
      Name: Actual filtered cloud (optional)
      Enabled: false
      Topic: /velodyne_points
      Queue Size: 1
      Style: Points
      Size (Pixels): 2
      Color Transformer: AxisColor
      Position Transformer: XYZ
  Enabled: true
  Global Options:
    Background Color: 35; 38; 45
    Fixed Frame: odom
    Frame Rate: 15
  Name: root
  Tools:
    - Class: rviz/Interact
    - Class: rviz/MoveCamera
    - Class: rviz/Select
    - Class: rviz/FocusCamera
    - Class: rviz/Measure
  Views:
    Current:
      Class: rviz/Orbit
      Distance: 3
      Focal Point:
        X: 0
        Y: 0
        Z: 0.65
      Pitch: 0.35
      Yaw: 0.8
Window Geometry:
  Width: 1200
  Height: 850
'''


README = '''实车待机 MPPI 独立版本 standby_20261105_01

这是 shadow（旁路观察）部署：读取真实状态和传感器，显示建议避障动作，
不会向 /cmd_vel、/joint_ctrl_single 或其他执行器发送指令，不启动底盘、
机械臂驱动，不自动使能。它不是已经放行的实车运动控制版本。

原环境保留：不修改 src、devel、.bashrc、原 launch 或系统 Python。
仅 source /opt/ros/noetic/setup.bash；使用 /usr/bin/python3，禁用用户 site。
core/model/config 冻结于本目录；运行日志、ROS_HOME 和 PID 文件只写本目录。
命名空间 /standby_trial_20261105_01。部署不需要 catkin_make 或 pip install。

操作（从本目录运行）：
1. bin/preflight.sh
   只读检查依赖、版本哈希、CAN/串口存在情况；不会打开设备。
2. bin/start_monitor.sh
   无现有 ROS 时，在 localhost:11461 启独立 roscore + shadow。
   没有真实输入时应显示等待状态，不伪造机器人、障碍或运动结果。
3. bin/start_shadow.sh --master-uri http://127.0.0.1:11311
   有真实传感器 ROS 图时，连接明确指定的现有 master；不启动任何驱动。
   远程 master 还需 --advertise-host <本机可达地址>。
4. bin/preflight.sh --master-uri http://127.0.0.1:11311
   额外保存真实 ROS 图、话题类型、/use_sim_time。全程只读。
5. bin/status.sh / bin/stop.sh
   仅检查/停止本 release 记录且 PID 身份匹配的进程组；不 killall，
   不停止别人的 master、驱动或原版控制器。
6. bin/view_shadow.sh
   打开本release的RViz。fixed frame为odom，只显示独立MarkerArray；
   原始/滤波实际点云可在Displays勾选，不提供导航目标或姿态发布工具。
   无传感器时只有等待提示，不生成虚构的实车运动。

A/B 对照：A 继续使用原来的启动入口和参数，不需要本目录。
B 目前是旁路影子组，适合对照建议轨迹/风险检测/计算时延，不能据此
声称完成真实避障。关闭 B 仅需 bin/stop.sh；原始文件与原环境不受改写。
执行模式还需独立网关、唯一控制权、真实时序/几何标定、现场确认后放行。

传感器配置见 config/hardware_shadow.json。待机锚点与机械臂保持姿态由
新鲜、静止的实测状态捕获。仿真姿态仅存在 config/simulation_source.json
用于溯源，默认不作为实车目标。机械臂关节速度从真实位置差分估计。
任何输出均在独立命名空间；输出可视化不等于已向真实机器人执行。
动态点云默认订阅本release的私有感知输出。已有真实传感器和定位数据后，
才可显式使用 bin/start_private_tracker.sh；该入口不自动启动任何硬件驱动。
私有slow-target参数用于观察0.08 m/s慢目标，仍未实车验收，不能据此实控。
显式入口：bin/start_private_tracker.sh --master-uri http://127.0.0.1:11311
该入口只连接已存在的master，前台运行；Ctrl-C仅停止这个私有感知进程。
远程master需另加 --advertise-host <本机可达地址>。它不启动任何底盘/机械臂
驱动、不使能机器人、不发布执行器命令；没有真实传感器时仍无实车数据。

几何仍为 native Piper DH 球模型，URDF 仅作冻结运动学与关节限位依据。
与真实连杆/工具/安装高度对齐尚未验收。model/mm.urdf 删除未使用的
visual/collision 网格与 Gazebo 元素，不依赖原环境mesh；转换来源有哈希。
joint6 上下界在冻结URDF与配置中收窄到[-0.35,0.35]弧度，与旧桥一致。
默认不加载 URDF 到全局 robot_description，不覆盖 TF 或传感器参数。

版本校验：config/release_manifest.json 记录冻结文件 SHA256。
deploy_local.py --refresh-manifest 仅在 runtime 编写完成后更新本 release
清单，不修改原代码。preflight 输出存 logs/preflight_*.json。
'''


def manifest():
    files={}
    for directory in ['bin','config','model','core','runtime','lib']:
        for path in sorted((RELEASE/directory).rglob('*')):
            if path.is_file() and '__pycache__' not in path.parts and path.name!='release_manifest.json':
                files[str(path.relative_to(RELEASE))]=digest(path)
    for name in ['README.txt','package.xml']:
        if (RELEASE/name).is_file():
            files[name]=digest(RELEASE/name)
    path=RELEASE/'config/release_manifest.json'
    # The manifest is intentionally refreshed after the separately owned runtime arrives.
    path.write_text(json.dumps(dict(schema=1,version='standby_20261105_01',
        namespace='/standby_trial_20261105_01',mode='shadow-only',hardware_motion_enabled=False,
        source_case=str(CASE.relative_to(WORKSPACE)),files=files),ensure_ascii=False,indent=2)+'\n')
    return len(files)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--refresh-manifest',action='store_true')
    args=parser.parse_args()
    if args.refresh_manifest:
        print(json.dumps(dict(release=str(RELEASE),frozen_files=manifest()))); return
    case=json.loads((CASE/'config.json').read_text())
    status=json.loads((CASE/'run_status.json').read_text())
    if not status.get('result_written') or status.get('required_process_failed'):
        raise RuntimeError('The source experiment is not a completed valid case')
    for name in ['active_arm_model.py','active_arm_mppi.py','standby_whole_body_mppi.py']:
        write(RELEASE/'core'/name,(CASE/'runtime'/name).read_bytes())
    write(RELEASE/'model/parameters.yaml',(CASE/'parameters.yaml').read_bytes())
    freeze_model()
    json_write(RELEASE/'config/limits.json',json.loads((CASE/'limits.json').read_text()))
    shadow=dict(schema=1,version='standby_20261105_01',namespace='/standby_trial_20261105_01',
        mode='shadow',hardware_motion_enabled=False,world_frame='odom',base_frame='base_link',
        topics=dict(odom='/odom',joints='/joint_states_single',raw_cloud='/livox/lidar_pcl2',
            filtered_cloud='/velodyne_points',dynamic_cloud='/standby_trial_20261105_01/perception/dynamic_points',
            perception_health='/standby_trial_20261105_01/perception/dynamic_obstacle_tracker/perception_healthy',
            static_cloud='/map_generator/global_cloud'),
        model=dict(parameters='model/parameters.yaml',urdf='model/mm.urdf'),
        max_age_s=.25,cloud_max_points=4000,static_max_points=15000,rate_hz=2.,
        horizon=case['horizon'],step=case.get('step',.1),random_samples=case['random_samples'],
        deadline_s=case['deadline_s'],seed=case['seed'],postures=[],kernel_config={},
        joint_limit_overrides=dict(joint6=[-.35,.35]))
    arm_limits={key:list(value) for key,value in case['arm_limits'].items()}
    arm_limits['lower'][5]=-.35; arm_limits['upper'][5]=.35
    shadow['arm_limits']=arm_limits
    json_write(RELEASE/'config/hardware_shadow.json',shadow)
    json_write(RELEASE/'config/simulation_source.json',dict(
        source_case=str(CASE.relative_to(WORKSPACE)),source_config_sha256=digest(CASE/'config.json'),
        scope='provenance only; these are NOT hardware position targets',
        q_start=case['q_start'],postures=case['postures'],arm_limits=case['arm_limits'],
        model_parameters_sha256=digest(CASE/'parameters.yaml')))
    write(RELEASE/'bin/environment.sh',ENV,True)
    write(RELEASE/'bin/session.py',SESSION,True)
    write(RELEASE/'bin/preflight.py',PREFLIGHT,True)
    for name,action in [('start_shadow','start-shadow'),('start_monitor','start-monitor'),
                        ('stop','stop'),('status','status'),('view_shadow','view')]:
        write(RELEASE/'bin'/(name+'.sh'), '#!/usr/bin/env bash\nset -euo pipefail\n'
            'source "$(dirname -- "${BASH_SOURCE[0]}")/environment.sh"\n'
            'exec /usr/bin/python3 "$STANDBY_RELEASE_DIR/bin/session.py" '+action+' "$@"\n',True)
    write(RELEASE/'bin/preflight.sh','#!/usr/bin/env bash\nset -euo pipefail\n'
        'source "$(dirname -- "${BASH_SOURCE[0]}")/environment.sh"\n'
        'exec /usr/bin/python3 "$STANDBY_RELEASE_DIR/bin/preflight.py" "$@"\n',True)
    write(RELEASE/'config/shadow.rviz',RVIZ)
    write(RELEASE/'README.txt',README)
    print(json.dumps(dict(release=str(RELEASE),frozen_files=manifest(),hardware_motion_enabled=False)))


if __name__=='__main__':
    main()
