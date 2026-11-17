#!/usr/bin/env python3
"""Isolated numerical whole-body plant; truth is sensor/audit only.

Only /sim/whole_body_command actuates this plant. Native trajectories and arm
commands are recorded, never executed. All dynamics use the sampler's advance
function, with <= 5 ms integration and independent physical-sphere truth audit.
"""
import csv
import hashlib
import io
import json
import math
import time
from pathlib import Path

import numpy as np

from active_arm_model import PiperArmModel, DEFAULT_PARAMETERS, DEFAULT_URDF
from active_arm_mppi import WholeBodyMPPI
from active_arm_scene import scene_definition, validate_scene, actors_3d_at, cylinder_gaps, obstacle_points

try:
    import rospy
    from simulator import Simulator, isolation, polynomial
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import JointState
    from sensor_msgs import point_cloud2 as pc2
    from std_msgs.msg import Header, Bool, Float64MultiArray
    from rosgraph_msgs.msg import Clock
    from quadrotor_msgs.msg import PolynomialTraj
    ROS_AVAILABLE = True
except ModuleNotFoundError:
    Simulator = object
    ROS_AVAILABLE = False


class WholeBodyPlant:
    """Pure numerical core. External truth never alters its command."""
    def __init__(self, model, limits, q_start, scene=None):
        self.model, self.limits = model, limits
        self.scene = scene_definition() if scene is None else scene
        validate_scene(self.scene)
        self.state = np.r_[np.zeros(5), np.asarray(q_start, dtype=float), np.zeros(6)]
        if self.state.shape != (17,) or not model.check(self.state[5:11])['valid']:
            raise ValueError('Initial arm state violates the declared model')
        self.dynamics = WholeBodyMPPI(model, limits,
            [model.lower, model.upper, model.velocity_limits, model.acceleration_limits],
            [], enabled=True)
        self.min_external_gap = float('inf'); self.min_configuration_gap = float('inf')
        self.external_gap = float('inf'); self.configuration_gap = float('inf')
        self.collided = False; self.joint_violation = False; self.first_violation = None
        self.arm_motion = np.zeros(6); self.arm_max_displacement = np.zeros(6)
        self.q_start = np.asarray(q_start, dtype=float).copy()
        self.max_substep = 0.; self.substeps = 0

    def audit(self, elapsed, sim_time=None):
        spheres = self.model.body_spheres(self.state[5:11], self.state[:3])
        xmin, xmax, ymin, ymax = self.scene['arena']
        walls = np.minimum.reduce([spheres[:, 0]-xmin-spheres[:, 3],
            xmax-spheres[:, 0]-spheres[:, 3], spheres[:, 1]-ymin-spheres[:, 3],
            ymax-spheres[:, 1]-spheres[:, 3]])
        obstacle = cylinder_gaps(spheres, actors_3d_at(elapsed, self.scene))
        self.external_gap = float(np.min(np.minimum(walls, obstacle)))
        self.configuration_gap = float(self.model.configuration_gaps_batch(self.state[None, 5:11])[0])
        self.min_external_gap = min(self.min_external_gap, self.external_gap)
        self.min_configuration_gap = min(self.min_configuration_gap, self.configuration_gap)
        joint_ok = all(value >= -1e-10 for value in
            self.model.joint_gaps(self.state[5:11], self.state[11:17]).values())
        self.joint_violation |= not joint_ok
        current_collision = self.external_gap < -1e-10 or self.configuration_gap < -1e-10
        self.collided |= current_collision
        if (current_collision or not joint_ok) and self.first_violation is None:
            self.first_violation = dict(t=sim_time, external_gap_m=self.external_gap,
                configuration_gap_m=self.configuration_gap, joint_limit_violation=not joint_ok,
                state=self.state.tolist())

    def step(self, command, dt, elapsed, sim_time=None):
        if not math.isfinite(dt) or not 0. < dt <= .005+1e-12:
            raise ValueError('Plant substep must be in (0, .005] seconds')
        u = np.asarray(command, dtype=float)
        if u.shape != (8,) or not np.isfinite(u).all():
            raise ValueError('Expected finite atomic base + six joint velocities')
        old_q = self.state[5:11].copy()
        self.state = self.dynamics.advance(self.state, u, dt=dt)
        self.arm_motion += np.abs(self.state[5:11]-old_q)
        self.arm_max_displacement = np.maximum(self.arm_max_displacement,
                                               np.abs(self.state[5:11]-self.q_start))
        self.max_substep = max(self.max_substep, dt); self.substeps += 1
        self.audit(elapsed, sim_time)


class ActiveArmSimulator(Simulator):
    def __init__(self):
        if not ROS_AVAILABLE: raise RuntimeError('ROS environment is required for simulator execution')
        super().__init__()
        self.config = json.loads((self.out/'active_arm_config.json').read_text())
        self.scene_spec = self.config.get('scene_definition', scene_definition())
        validate_scene(self.scene_spec)
        parameter_path = self.config.get('parameters_path', DEFAULT_PARAMETERS)
        self.model = PiperArmModel.from_parameters(parameter_path,
                          urdf_path=self.config.get('urdf_path', DEFAULT_URDF))
        arm_limits = self.config.get('arm_limits')
        if arm_limits is not None:
            if isinstance(arm_limits, dict):
                arm_limits = [arm_limits[k] for k in ('lower', 'upper', 'velocity', 'acceleration')]
            if len(arm_limits) != 4: raise ValueError('Expected four arm-limit vectors')
            lower, upper, velocity, acceleration = [np.broadcast_to(np.asarray(v, dtype=float), (6,)).copy() for v in arm_limits]
            if not np.isfinite(np.r_[lower, upper, velocity, acceleration]).all() or np.any(lower >= upper) or np.any(velocity <= 0) or np.any(acceleration <= 0):
                raise ValueError('Invalid configured arm limits')
            if np.any(lower < self.model.lower-1e-10) or np.any(upper > self.model.upper+1e-10):
                raise ValueError('Experiment bounds cannot exceed URDF joint limits')
            self.model.lower, self.model.upper = lower, upper
            self.model.velocity_limits, self.model.acceleration_limits = velocity, acceleration
        self.goal_xy = np.asarray(self.config.get('goal', self.scene_spec['goal']), dtype=float)
        if self.goal_xy.shape != (2,) or not np.isfinite(self.goal_xy).all(): raise ValueError('Invalid goal')
        self.duration = float(self.config.get('duration', self.duration))
        if not math.isfinite(self.duration) or self.duration <= 0: raise ValueError('Invalid trial duration')
        self.plant = WholeBodyPlant(self.model, self.limits,
                                   self.config.get('q_start', self.scene_spec['q_start']), self.scene_spec)
        self.q = self.plant.state[5:11].copy(); self.dq = np.zeros(6)
        self.whole_command = np.zeros(8); self.command_lease = .25
        self.command_time = -1e9; self.command_sequence = -1
        self.command_accepted = 0; self.command_rejected = 0; self.native_arm_commands_ignored = 0
        self.last_command_fresh = False; self.command_expiry_transitions = 0
        self.scene = self.scene_spec['scene']; self.scenario_start = 'motion'
        self.plant.audit(-1., self.t)
        rospy.Subscriber('/sim/whole_body_command', Float64MultiArray, self.whole_body_command, queue_size=1)

    def cmd(self, message):
        pass  # Legacy base command does not own this actuator.

    def local_cmd(self, message):
        pass

    def arm(self, message):
        self.native_arm_commands_ignored = getattr(self, 'native_arm_commands_ignored', 0)+1

    def whole_body_command(self, message):
        wire = np.asarray(message.data, dtype=float)
        with self.lock:
            if (wire.shape != (12,) or not np.isfinite(wire).all() or wire[0] != 1. or
                not 0. < wire[2] <= .25 or wire[3] != math.floor(wire[3]) or
                wire[3] <= self.command_sequence or wire[1]-1000 < self.command_time or
                not -.05 <= 1000+self.t-wire[1] < wire[2]):
                self.command_rejected += 1
                return
            self.whole_command = wire[4:].copy()
            self.command = wire[4:6].copy()
            self.command_time = wire[1]-1000; self.command_lease = wire[2]
            self.command_sequence = int(wire[3]); self.command_accepted += 1

    def trajectory(self, message):
        # Original serializer is retained as an audit; q differences are valid
        # references now, so its obsolete fixed-arm error accumulator is reset.
        super().trajectory(message)
        self.arm_error = 0.

    def integrate(self, dt):
        if dt <= 0: return
        count = max(1, int(math.ceil(dt/.005))); step = dt/count
        for index in range(count):
            subtime = self.t-dt+(index+1)*step
            with self.lock:
                fresh = -.05 <= subtime-self.command_time < self.command_lease
                command = self.whole_command.copy() if fresh else np.zeros(8)
            if self.last_command_fresh and not fresh: self.command_expiry_transitions += 1
            self.last_command_fresh = fresh
            elapsed = -1. if self.motion_time is None else subtime-self.motion_time
            self.plant.step(command, step, elapsed, subtime)
        self.s = self.plant.state[:5].copy()
        self.q = self.plant.state[5:11].copy(); self.dq = self.plant.state[11:17].copy()
        self.min_clearance = self.plant.min_external_gap; self.collided = self.plant.collided

    def publish_state(self):
        stamp = rospy.Time.from_sec(1000+self.t); self.clock_pub.publish(Clock(stamp))
        odom = Odometry(); odom.header = Header(stamp=stamp, frame_id='odom'); odom.child_frame_id='base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.s[:2]
        odom.pose.pose.orientation.z = math.sin(self.s[2]/2); odom.pose.pose.orientation.w = math.cos(self.s[2]/2)
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = self.s[3:]; self.odom_pub.publish(odom)
        transform = TransformStamped(); transform.header = odom.header; transform.child_frame_id='base_link'
        transform.transform.translation.x, transform.transform.translation.y = self.s[:2]
        transform.transform.rotation = odom.pose.pose.orientation; self.tf.sendTransform(transform)
        joints = JointState(); joints.header = Header(stamp=stamp)
        joints.name = ['joint'+str(i+1) for i in range(6)]
        joints.position = self.q.tolist(); joints.velocity = self.dq.tolist(); joints.effort = [0.]*6
        self.joint_pub.publish(joints); self.gripper_pub.publish(Bool(False))
        return stamp

    def cloud(self, stamp):
        elapsed = -1. if self.motion_time is None else self.t-self.motion_time
        world = np.concatenate([self.static, obstacle_points(elapsed, self.scene_spec)])
        delta = world-np.array([self.s[0], self.s[1], 0.]); c, s = math.cos(self.s[2]), math.sin(self.s[2])
        rotation = np.array([[c,s,0.],[-s,c,0.],[0.,0.,1.]])
        local = delta@rotation.T-self.offset
        local = local[np.linalg.norm(local[:, :2], axis=1) < 5.]
        self.raw_pub.publish(pc2.create_cloud_xyz32(Header(stamp=stamp, frame_id='livox_frame'), local))

    def record(self):
        self.rows.append([self.t, *self.s, *self.command, self.limit,
            self.plant.external_gap, self.traj_count, self.dynamic_n, int(self.health),
            *self.progress, 0., *self.q, *self.dq, *self.whole_command[2:],
            self.plant.configuration_gap, int(self.last_command_fresh)])
        elapsed = -1. if self.motion_time is None else self.t-self.motion_time
        for actor in actors_3d_at(elapsed, self.scene_spec):
            self.obstacle_rows.append([self.t]+[actor[k] for k in ('x','y','r','id','vx','vy','z_min','z_max')])

    def write_results(self, reason):
        header = ('t,x,y,yaw,v,w,cmd_v,cmd_w,safety_limit,clearance,trajectory_count,dynamic_points,'
            'perception_healthy,trajectory_start,virtual_time,time_scale,arm_error').split(',')
        header += ['q'+str(i+1) for i in range(6)]+['dq'+str(i+1) for i in range(6)]
        header += ['cmd_dq'+str(i+1) for i in range(6)]+['configuration_gap','whole_body_command_fresh']
        for name, columns, rows in [('state',header,self.rows),
             ('obstacles','t,x,y,r,id,vx,vy,z_min,z_max'.split(','),self.obstacle_rows)]:
            with (self.out/(name+'.csv')).open('w') as stream:
                writer=csv.writer(stream); writer.writerow(columns); writer.writerows(rows)
        for name, value in [('events',self.events),('trajectories',self.raw_trajs),('mppi_diagnostics',self.diagnostics)]:
            (self.out/(name+'.json')).write_text(json.dumps(value,indent=2)+'\n')
        result = dict(reason=reason, goal_reached=reason=='goal', goal_sent=self.goal_time,
            motion_anchor=self.motion_time, scenario_start='motion', trajectory_bundles=self.traj_count,
            min_clearance=self.plant.min_external_gap, min_external_gap_m=self.plant.min_external_gap,
            min_configuration_gap_m=self.plant.min_configuration_gap,
            collision_physical=self.plant.collided, joint_limit_violation=self.plant.joint_violation,
            first_violation=self.plant.first_violation, final_state=self.s.tolist(),
            final_q=self.q.tolist(), final_dq=self.dq.tolist(), elapsed=self.t, scene=self.scene,
            arm_max_displacement_rad=self.plant.arm_max_displacement.tolist(),
            arm_total_motion_rad=self.plant.arm_motion.tolist(),
            command_accepted=self.command_accepted, command_rejected=self.command_rejected,
            command_expiry_transitions=self.command_expiry_transitions,
            native_arm_commands_ignored=self.native_arm_commands_ignored,
            maximum_integration_step_s=self.plant.max_substep, audit_substeps=self.plant.substeps,
            collision_pair_policy=self.model.collision_policy(),
            scope='Native DH/sphere numerical ROS plant; 5 ms sampled audits; no mesh/continuous/hardware certification')
        (self.out/'result.json').write_text(json.dumps(result,indent=2)+'\n')

    def run(self):
        start=time.monotonic(); last=start; next_cloud=0.; next_map=0.; settled=0.; reason='shutdown'
        try:
            while not rospy.is_shutdown():
                now=time.monotonic(); self.t=now-start; dt=now-last; last=now
                if dt>.25: self.events.append(dict(t=self.t,kind='plant_loop_gap',dt=dt))
                self.integrate(dt); stamp=self.publish_state()
                if self.motion_time is None and self.goal_time is not None and self.s[0]>=.10 and self.s[3]>=.05:
                    self.motion_time=self.t; self.events.append(dict(t=self.t,kind='motion_anchor',x=float(self.s[0]),v=float(self.s[3])))
                if self.t>=next_cloud: self.cloud(stamp); next_cloud=self.t+.1
                if self.t>=next_map:
                    self.map_pub.publish(pc2.create_cloud_xyz32(Header(stamp=stamp,frame_id='world'),self.static)); next_map=float('inf')
                if self.goal_time is None and self.t>7 and self.trigger and self.goal_pub.get_num_connections():
                    goal=PoseStamped(); goal.header=Header(stamp=stamp,frame_id='world')
                    goal.pose.position.x,goal.pose.position.y=self.goal_xy; goal.pose.orientation.w=1.
                    self.goal_pub.publish(goal); self.goal_time=self.t; self.events.append(dict(t=self.t,kind='goal'))
                self.record()
                if self.plant.collided: reason='collision'; break
                if self.plant.joint_violation: reason='joint_limit_violation'; break
                at_goal=(self.goal_time is not None and np.linalg.norm(self.s[:2]-self.goal_xy)<self.limits['goal_pos']
                    and abs(self.s[2])<self.limits['goal_yaw'] and abs(self.s[3])<self.limits['goal_speed']
                    and np.max(np.abs(self.dq))<.02)
                settled=settled+dt if at_goal else 0.
                if settled>1.: reason='goal'; break
                if self.t>(self.goal_time+self.duration if self.goal_time is not None else 20.): reason='timeout'; break
                time.sleep(max(0.,.02-(time.monotonic()-now)))
        finally:
            self.write_results(reason)


if __name__ == '__main__':
    if not ROS_AVAILABLE: raise RuntimeError('Source the isolated ROS workspace before running')
    isolation(); rospy.init_node('active_arm_simulator')
    if not rospy.get_param('/online_validation/enabled',False) or not rospy.get_param('/use_sim_time',False):
        raise RuntimeError('Missing simulation interlock')
    ActiveArmSimulator().run()
