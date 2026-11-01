#!/usr/bin/env python3
"""ROS-free identity, atomic snapshot and optional diagnostic I/O regressions."""
import contextlib
import copy
import importlib.util
import io
import json
import math
from pathlib import Path
import struct
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from state_contract_records import (IDENTITY_ALGORITHM, JsonlCapture, capture_enabled,
    copy_state_and_identity, fnv1a64, identity_or_error, odom_identity,
    record_with_validity)


def vector(x=0., y=0., z=0.):
    return SimpleNamespace(x=x, y=y, z=z)


class FakeOdometry:
    """A deliberately small ROS1-style message with binary serialization."""
    def __init__(self):
        self.header=SimpleNamespace(seq=7, stamp=SimpleNamespace(secs=1002, nsecs=123456789),
                                    frame_id='world')
        self.child_frame_id='base_link'
        self.pose=SimpleNamespace(pose=SimpleNamespace(position=vector(1., 2., .3),
            orientation=SimpleNamespace(x=0., y=0., z=math.sin(.2), w=math.cos(.2))))
        self.twist=SimpleNamespace(twist=SimpleNamespace(linear=vector(.15, .01, .02),
                                                        angular=vector(.03, .04, -.1)))

    def serialize(self, stream):
        h=self.header; p=self.pose.pose; t=self.twist.twist
        stream.write(struct.pack('<III', h.seq, h.stamp.secs, h.stamp.nsecs))
        for value in (h.frame_id, self.child_frame_id):
            raw=value.encode('utf-8');stream.write(struct.pack('<I', len(raw)));stream.write(raw)
        stream.write(struct.pack('<13d', p.position.x, p.position.y, p.position.z,
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w,
            t.linear.x, t.linear.y, t.linear.z, t.angular.x, t.angular.y, t.angular.z))


class StateContractRecords(unittest.TestCase):
    def test_standard_fnv1a_known_vectors_and_serialized_identity(self):
        self.assertEqual(fnv1a64(b''), '14695981039346656037')
        self.assertEqual(fnv1a64(b'a'), str(0xaf63dc4c8601ec8c))
        self.assertEqual(fnv1a64(b'hello'), str(0xa430d84680aabd0b))
        message=FakeOdometry(); raw=io.BytesIO();message.serialize(raw)
        identity=odom_identity(message)
        self.assertEqual(identity['algorithm'], IDENTITY_ALGORITHM)
        self.assertEqual(identity['message_hash'], fnv1a64(raw.getvalue()))
        self.assertEqual(identity['serialized_bytes'], len(raw.getvalue()))
        self.assertEqual((identity['header_seq'], identity['stamp_sec'], identity['stamp_nsec']),
                         (7, 1002, 123456789))
        self.assertEqual(identity['frame_id'], 'world')
        self.assertEqual(identity['child_frame_id'], 'base_link')
        self.assertAlmostEqual(identity['yaw'], .4)
        self.assertEqual(identity['body_twist_linear'], [.15, .01, .02])
        self.assertEqual(identity['body_twist_angular'], [.03, .04, -.1])
        self.assertTrue(identity['validity']['state_finite'])

    def test_same_stamp_different_payload_is_not_identity_match(self):
        first=FakeOdometry();second=FakeOdometry();second.pose.pose.position.x+=.000001
        a=odom_identity(first); b=odom_identity(second)
        self.assertEqual((a['stamp_sec'], a['stamp_nsec']), (b['stamp_sec'], b['stamp_nsec']))
        self.assertNotEqual(a['message_hash'], b['message_hash'])
        second=FakeOdometry();second.header.seq+=1
        self.assertNotEqual(a['message_hash'], odom_identity(second)['message_hash'])

    def test_nonfinite_is_null_with_explicit_validity_not_zero(self):
        message=FakeOdometry();message.twist.twist.linear.x=float('nan')
        message.pose.pose.position.z=float('inf')
        identity=odom_identity(message)
        self.assertIsNone(identity['state'][3]);self.assertIsNone(identity['position'][2])
        self.assertFalse(identity['validity']['state_finite'])
        self.assertFalse(identity['validity']['all_numeric_finite'])
        self.assertIn('$.state[3]', identity['validity']['nonfinite_paths'])
        row=record_with_validity({'state':[float('-inf'), 0.], 'identity':identity})
        self.assertIsNone(row['state'][0]);self.assertEqual(row['state'][1], 0.)
        self.assertEqual(row['numeric_validity']['nonfinite_paths'], ['$.state[0]'])
        json.dumps(row, allow_nan=False)

    def test_state_copy_and_identity_are_equivalent_and_detached(self):
        identity=odom_identity(FakeOdometry());state=identity['state'].copy()
        copied_state,copied_identity=copy_state_and_identity(state,identity)
        self.assertEqual(copied_state,copied_identity['state'])
        self.assertEqual(copied_identity,identity)
        state[0]=900.;identity['position'][0]=800.;identity['validity']['nonfinite_paths'].append('changed')
        self.assertEqual(copied_state[0],1.);self.assertEqual(copied_identity['position'][0],1.)
        self.assertEqual(copied_identity['validity']['nonfinite_paths'],[])
        self.assertEqual(copy_state_and_identity(None,None),(None,None))

    def test_capture_requires_exact_opt_in_and_disabled_never_opens_file(self):
        for setting in ('', '0', 'true', 'yes'):
            with patch.dict('os.environ', {'STATE_CONTRACT_CAPTURE':setting}):
                self.assertFalse(capture_enabled())
                with patch.object(Path, 'open', side_effect=AssertionError('disabled I/O')):
                    sink=JsonlCapture(Path('/not-created/state.jsonl'))
                    self.assertFalse(sink.write({'state':[1.]}));sink.close()
        with patch.dict('os.environ', {'STATE_CONTRACT_CAPTURE':'1'}):
            self.assertTrue(capture_enabled())

    def test_enabled_sink_sanitizes_and_io_failure_cannot_raise_into_control(self):
        with tempfile.TemporaryDirectory(prefix='state_contract_records_') as directory:
            path=Path(directory)/'cycles.jsonl';sink=JsonlCapture(path, enabled=True)
            self.assertTrue(sink.write({'command_v':float('nan')}));sink.close()
            row=json.loads(path.read_text())
            self.assertIsNone(row['command_v']);self.assertFalse(row['numeric_validity']['all_finite'])
        errors=io.StringIO()
        with patch.object(Path,'open',side_effect=OSError('read-only diagnostic path')),contextlib.redirect_stderr(errors):
            sink=JsonlCapture(Path('/not-created/state.jsonl'),enabled=True)
            self.assertFalse(sink.write({'command_v':.2}));sink.close()
        self.assertIn('STATE_CONTRACT_CAPTURE_ERROR',errors.getvalue())
        self.assertIn('read-only diagnostic path',errors.getvalue())
        broken_stderr=Mock(write=Mock(side_effect=BrokenPipeError('stderr closed')))
        with patch.object(Path,'open',side_effect=OSError('path closed')),patch('sys.stderr',broken_stderr):
            sink=JsonlCapture(Path('/not-created/state.jsonl'),enabled=True)
        sink.stream=Mock(closed=False,write=Mock(side_effect=OSError('disk full')),
                         close=Mock(side_effect=OSError('close failed')))
        with patch('sys.stderr',broken_stderr):
            self.assertFalse(sink.write({'command_v':.2}));sink.close()

    def test_serialization_error_is_explicit_not_a_fake_identity(self):
        message=FakeOdometry()
        with patch.object(message,'serialize',side_effect=ValueError('broken serialization')):
            identity=identity_or_error(message)
        self.assertIsNone(identity['message_hash'])
        self.assertFalse(identity['validity']['serialized'])
        self.assertIn('broken serialization', identity['capture_error'])

    def test_observer_records_every_callback_before_existing_motion_throttle(self):
        import dual_timescale_observer as observer_module
        with tempfile.TemporaryDirectory(prefix='state_contract_observer_') as directory:
            observer=observer_module.Observer.__new__(observer_module.Observer)
            observer.lock=threading.RLock();observer.last_odom=0.;observer.log=io.StringIO()
            observer.rospy=SimpleNamespace(get_time=lambda:1002.1)
            path=Path(directory)/'state_odom_events.jsonl'
            observer.state_odom_capture=JsonlCapture(path,enabled=True)
            message=FakeOdometry();stamp=message.header.stamp
            stamp.to_sec=lambda:stamp.secs+stamp.nsecs*1e-9
            with patch.object(observer_module.time,'monotonic',return_value=10.):
                observer.odom(message)
                message.pose.pose.position.x+=.1
                observer.odom(message)
            observer.state_odom_capture.close()
            rows=[json.loads(line) for line in path.read_text().splitlines()]
            self.assertEqual(len(rows),2)
            self.assertEqual(len(observer.log.getvalue().splitlines()),1)
            self.assertNotEqual(rows[0]['odom_identity']['message_hash'],rows[1]['odom_identity']['message_hash'])
            self.assertEqual(rows[0]['callback_ros_time'],1002.1)
            self.assertEqual(rows[0]['callback_monotonic'],10.)

    def test_actual_cycle_copies_both_identities_and_records_after_publication(self):
        # Import the real run loop with inert message classes and an in-process
        # kernel double. No ROS installation, node, topic or graph is involved.
        import numpy as np
        def header():return SimpleNamespace(frame_id='', stamp=0.)
        class Pose:
            def __init__(self):
                self.header=header();self.pose=SimpleNamespace(position=vector(),
                    orientation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.))
        class Twist:
            def __init__(self):self.linear=vector();self.angular=vector()
        class Stamped:
            def __init__(self):self.header=header();self.twist=Twist();self.vector=vector()
        class RosPath:
            def __init__(self):self.header=header();self.poses=[]
        geometry=SimpleNamespace(PoseStamped=Pose,Twist=Twist,TwistStamped=Stamped,Vector3Stamped=Stamped)
        messages={'geometry_msgs':SimpleNamespace(msg=geometry),'geometry_msgs.msg':geometry,
            'nav_msgs':SimpleNamespace(), 'nav_msgs.msg':SimpleNamespace(Path=RosPath),
            'sensor_msgs':SimpleNamespace(), 'sensor_msgs.msg':SimpleNamespace(JointState=object),
            'std_msgs':SimpleNamespace(), 'std_msgs.msg':SimpleNamespace(String=lambda value:SimpleNamespace(data=value)),
            'quadrotor_msgs':SimpleNamespace(), 'quadrotor_msgs.msg':SimpleNamespace(PolynomialTraj=object)}
        controls=[];captures=[]
        for enabled,blocked in ((False,False),(True,False),(True,True)):
            sleeps=[];commands=[];records=[];kernel_marks=[];publish_marks=[];write_marks=[]
            clocks={'ros':1002.1,'mono':10.}
            def ros_clock():
                clocks['ros']+=1e-6
                return clocks['ros']
            def mono_clock():
                clocks['mono']+=1e-6
                return clocks['mono']
            def publish_command(cmd):
                publish_marks.append((clocks['ros'],clocks['mono']))
                commands.append([cmd.linear.x,cmd.angular.z])
            fake_ros=SimpleNamespace(Rate=lambda _:SimpleNamespace(sleep=lambda:sleeps.append(1)),
                is_shutdown=lambda:len(sleeps)>=2,get_time=ros_clock,
                Time=SimpleNamespace(from_sec=lambda value:value),ROSInterruptException=RuntimeError)
            spec=importlib.util.spec_from_file_location('_state_contract_local_execution',
                Path(__file__).with_name('local_execution.py'))
            module=importlib.util.module_from_spec(spec)
            with patch.dict('sys.modules',dict(messages,rospy=fake_ros)):
                spec.loader.exec_module(module)
            message=FakeOdometry();message.header.stamp.nsecs=0
            identity=odom_identity(message)
            guide=SimpleNamespace(epoch=2,revision=3,messages=['exact-native-message'],start=1002.,
                refs=[np.array([[1.,2.,.4,.15],[2.,2.,.4,.15]])],times=[np.array([0.,1.])])
            session=SimpleNamespace(generation=7,guide=guide,revision=3,guide_cancelled=False,
                invalid=None,authority=lambda:'native_hard_stop' if blocked else 'ok')
            node=SimpleNamespace(lock=threading.Lock(),execution=session,pending_execution_transactions=[],
                state=np.array(identity['state']),odom_identity=identity,odom_stamp=1002.,
                field=np.full((2,2),10.),obs=np.empty((0,6)),cloud_stamp=1002.,obs_stamp=1002.,
                healthy=True,arm_stamp=1002.,arm_error=0.,shared_collision=False,classified_error=None,
                classification={},strategy_debug=False,execution_alternatives=False,handle=1,
                nx=2,ny=2,gx=0.,gy=0.,res=.05,reference_log=io.StringIO(),execution_log=io.StringIO(),
                reference_pub=SimpleNamespace(publish=lambda _:None),progress_pub=None,stall_streak=0,
                saved_failures=set(),safety=-1.,search_recorder=None,
                diag=SimpleNamespace(publish=lambda _:None),execution_pub=SimpleNamespace(publish=lambda _:None),
                local_cmd_pub=SimpleNamespace(publish=lambda _:None),
                pub=SimpleNamespace(publish=publish_command))
            initial_identity=copy.deepcopy(identity)
            class Kernel:
                def online_step(self,*args):
                    kernel_marks.append((clocks['ros'],clocks['mono']))
                    self.solve=np.ctypeslib.as_array(args[1],shape=(5,)).copy()
                    output=np.ctypeslib.as_array(args[-1],shape=(8,));output[:4]=[.12,.01,.12,.01]
                    # Model a callback arriving between solve and validation,
                    # including a changed payload with an identical ROS stamp.
                    with node.lock:
                        message.pose.pose.position.x+=.1
                        node.odom_identity=odom_identity(message)
                        node.state=np.array(node.odom_identity['state'])
                    return 0
                def online_selected_controls(self,handle,output,count):
                    if output is None:return 2
                    np.ctypeslib.as_array(output,shape=(4,))[:]=[.12,.01,.12,.01]
                    return 2
                def online_validate_execution(self,*args):
                    output=np.ctypeslib.as_array(args[-1],shape=(6,));output[:]=[1.,1.,1.,11.,1.,1.]
                    return 0
                def online_reset(self,*_):pass
                def online_reference_changed(self,*_):pass
                def online_decision_debug(self,*_):pass
                def online_recovery_debug(self,*_):pass
                def online_feedback(self,*_):pass
                def online_destroy(self,*_):pass
            class Capture:
                def __init__(self):self.enabled=True
                def write(self,row):
                    self.assert_outside_lock=not node.lock.locked()
                    write_marks.append((clocks['ros'],clocks['mono']))
                    records.append(record_with_validity(row))
                    self.published_before_write=len(commands)
                    return True
                def close(self):pass
            node.lib=Kernel()
            if enabled:node.state_contract_cycles=Capture()
            with patch.object(module.time,'monotonic',side_effect=mono_clock):module.run(node)
            controls.append(commands);captures.append(records)
            if enabled:
                self.assertTrue(node.state_contract_cycles.assert_outside_lock)
                self.assertEqual(node.state_contract_cycles.published_before_write,2)
                for index,row in enumerate(records):
                    for dimension,suffix in enumerate(('ros_time','monotonic')):
                        before=row['command_publish_before_'+suffix]
                        after=row['command_publish_after_'+suffix]
                        self.assertLessEqual(before,publish_marks[index][dimension])
                        self.assertLessEqual(publish_marks[index][dimension],after)
                        self.assertLessEqual(after,write_marks[index][dimension])
                    if blocked:
                        self.assertIsNone(row['online_step_entry_ros_time'])
                        self.assertIsNone(row['online_step_entry_monotonic'])
                        self.assertFalse(row['online_step_invoked'])
                    else:
                        self.assertLessEqual(row['solve_ros_time'],row['online_step_entry_ros_time'])
                        self.assertLessEqual(row['online_step_entry_ros_time'],kernel_marks[index][0])
                        self.assertLessEqual(row['online_step_entry_monotonic'],kernel_marks[index][1])
                        self.assertLess(kernel_marks[index][0],row['validation_snapshot_ros_time'])
                        self.assertLessEqual(row['command_ros_time'],row['command_publish_before_ros_time'])
                if blocked:
                    self.assertFalse(kernel_marks)
                    self.assertTrue(all(row['reason']=='native_hard_stop' for row in records))
                    self.assertEqual(commands,[[0.,0.],[0.,0.]])
                    continue
                self.assertEqual(records[0]['solve_odom_identity'],initial_identity)
                self.assertNotEqual(records[0]['solve_odom_identity']['message_hash'],
                                    records[0]['validation_odom_identity']['message_hash'])
                self.assertEqual(records[0]['solve_state'],records[0]['solve_odom_identity']['state'])
                self.assertEqual(records[0]['validation_state'],records[0]['validation_odom_identity']['state'])
                self.assertEqual(records[0]['validation_odom_identity'],records[1]['solve_odom_identity'])
                self.assertEqual([r['cycle_id'] for r in records],[1,2])
                self.assertEqual([r['first_kernel_cycle_for_reference'] for r in records],[True,False])
                self.assertEqual([r['first_validated_command_cycle_for_reference'] for r in records],[True,False])
                self.assertTrue(all(r['online_step_invoked'] and r['online_step_succeeded'] for r in records))
                self.assertTrue(all(r['validation_invoked'] and r['validated_command_published'] for r in records))
        self.assertEqual(controls[0],controls[1]);self.assertEqual(controls[1],[[.12,.01],[.12,.01]])
        self.assertEqual(captures[0],[])


if __name__=='__main__':
    unittest.main()
