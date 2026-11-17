#!/usr/bin/env python3
"""Free whole-body numerical plant with a sudden arm-height rectangular box.

Both control groups share the original arm/base dynamics. Ground truth enters
only synthetic sensing and the independent physical-sphere collision auditor.
"""
import csv
import json
import math

import numpy as np

from active_arm_model import PiperArmModel, DEFAULT_PARAMETERS, DEFAULT_URDF
from active_arm_simulator import ActiveArmSimulator, WholeBodyPlant, ROS_AVAILABLE
from arm_box_scene import (scene_definition, validate_scene, place_box,
                           box_gaps, obstacle_points, straight_path_precheck)

if ROS_AVAILABLE:
    import rospy
    from control_msgs.msg import JointTrajectoryControllerState
    from geometry_msgs.msg import Twist
    from sensor_msgs import point_cloud2 as pc2
    from std_msgs.msg import Header, Float64MultiArray
    from simulator import Simulator, isolation


class BoxWholeBodyPlant(WholeBodyPlant):
    """Identical dynamics, with the box's exact finite AABB audit geometry."""
    def __init__(self, model, limits, q_start, scene):
        validate_scene(scene)
        # The parent constructor only validates legacy metadata; no cylindrical
        # truth is retained or passed to the box auditor.
        bridge = dict(scene, schema='active_arm_scene_v1', actors=[])
        super().__init__(model, limits, q_start, bridge)
        self.scene = scene
        self.box = None
        self.base_box_gap = self.arm_box_gap = float('inf')
        self.min_base_box_gap = self.min_arm_box_gap = float('inf')

    def audit(self, elapsed, sim_time=None):
        spheres = self.model.body_spheres(self.state[5:11], self.state[:3])
        xmin, xmax, ymin, ymax = self.scene['arena']
        walls = np.minimum.reduce([spheres[:, 0]-xmin-spheres[:, 3],
            xmax-spheres[:, 0]-spheres[:, 3], spheres[:, 1]-ymin-spheres[:, 3],
            ymax-spheres[:, 1]-spheres[:, 3]])
        obstacle = box_gaps(spheres, self.box)
        fixed_n = len(self.model.fixed_spheres)
        self.base_box_gap = float(obstacle[:fixed_n].min())
        self.arm_box_gap = float(obstacle[fixed_n:].min())
        self.min_base_box_gap = min(self.min_base_box_gap, self.base_box_gap)
        self.min_arm_box_gap = min(self.min_arm_box_gap, self.arm_box_gap)
        self.external_gap = float(np.minimum(walls, obstacle).min())
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
                base_box_gap_m=self.base_box_gap, arm_box_gap_m=self.arm_box_gap,
                state=self.state.tolist())


class ArmBoxSimulator(ActiveArmSimulator):
    def __init__(self):
        if not ROS_AVAILABLE:
            raise RuntimeError('Source the isolated ROS environment first')
        # Simulator owns the same topics, static wall cloud and recording state.
        self.native_base_count = 0
        self.native_arm_count = 0
        self.native_pose = np.zeros(3)
        self.native_velocity = np.zeros(2)
        self.native_q = np.zeros(6)
        self.native_dq = np.zeros(6)
        self.native_stamp = self.joint_stamp = -1e9
        self.group = ''
        Simulator.__init__(self)
        self.config = json.loads((self.out/'active_arm_config.json').read_text())
        self.group = self.config['group']
        if self.group not in ('native', 'mppi', 'fast_only'):
            raise ValueError('Unknown arm-box group')
        self.scene_spec = self.config.get('scene_definition', scene_definition())
        validate_scene(self.scene_spec)
        self.model = PiperArmModel.from_parameters(self.config.get('parameters_path', DEFAULT_PARAMETERS),
            urdf_path=self.config.get('urdf_path', DEFAULT_URDF))
        arm_limits = self.config.get('arm_limits')
        if arm_limits is not None:
            if isinstance(arm_limits, dict):
                arm_limits = [arm_limits[key] for key in ('lower', 'upper', 'velocity', 'acceleration')]
            if len(arm_limits) != 4:
                raise ValueError('Expected four arm-limit vectors')
            lower, upper, velocity, acceleration = [np.broadcast_to(np.asarray(value, dtype=float), (6,)).copy()
                for value in arm_limits]
            if (not np.isfinite(np.r_[lower, upper, velocity, acceleration]).all() or np.any(lower >= upper)
                or np.any(velocity <= 0) or np.any(acceleration <= 0)
                or np.any(lower < self.model.lower-1e-10) or np.any(upper > self.model.upper+1e-10)):
                raise ValueError('Invalid configured arm bounds')
            self.model.lower, self.model.upper = lower, upper
            self.model.velocity_limits, self.model.acceleration_limits = velocity, acceleration
        self.goal_xy = np.asarray(self.config.get('goal', self.scene_spec['goal']), dtype=float)
        if self.goal_xy.shape != (2,) or not np.isfinite(self.goal_xy).all():
            raise ValueError('Invalid goal')
        self.duration = float(self.config.get('duration', self.duration))
        if not math.isfinite(self.duration) or self.duration <= 0:
            raise ValueError('Invalid trial duration')
        self.plant = BoxWholeBodyPlant(self.model, self.limits,
            self.config.get('q_start', self.scene_spec['q_start']), self.scene_spec)
        self.q = self.plant.state[5:11].copy()
        self.dq = np.zeros(6)
        self.native_q = self.q.copy()
        self.whole_command = np.zeros(8)
        self.command_lease = .25
        self.command_time = -1e9
        self.command_sequence = -1
        self.command_accepted = self.command_rejected = 0
        self.native_arm_commands_ignored = 0
        self.last_command_fresh = False
        self.command_expiry_transitions = 0
        self.scene = self.scene_spec['scene']
        self.scenario_start = 'motion'
        self.box_runtime = None
        self.plant.audit(-1., self.t)
        rospy.Subscriber('/sim/whole_body_command', Float64MultiArray, self.whole_body_command, queue_size=1)
        rospy.Subscriber('/mm_controller_node/car_cmd', Twist, self.native_base, queue_size=1)

    def whole_body_command(self, message):
        if self.group != 'native':
            super().whole_body_command(message)

    def native_base(self, message):
        if self.group != 'native':
            return
        pose = np.array([message.linear.x, message.linear.y, message.linear.z])
        velocity = np.array([message.angular.x, message.angular.y])
        if not np.isfinite(np.r_[pose, velocity]).all():
            return
        with self.lock:
            self.native_pose, self.native_velocity = pose, velocity
            self.native_stamp = self.t
            self.native_base_count += 1

    def arm(self, message):
        if self.group != 'native':
            self.native_arm_commands_ignored = getattr(self, 'native_arm_commands_ignored', 0)+1
            return
        q = np.asarray(message.desired.positions, dtype=float)
        dq = np.asarray(message.desired.velocities, dtype=float)
        if q.shape != (6,) or dq.shape != (6,) or not np.isfinite(np.r_[q, dq]).all():
            return
        with self.lock:
            self.native_q, self.native_dq = q, dq
            self.joint_stamp = self.t
            self.native_arm_count += 1

    def controls(self, now):
        with self.lock:
            if self.group != 'native':
                fresh = -.05 <= now-self.command_time < self.command_lease
                return (self.whole_command.copy() if fresh else np.zeros(8)), fresh
            state = self.plant.state
            command = np.zeros(8)
            base_fresh = -.05 <= now-self.native_stamp < .3
            arm_fresh = -.05 <= now-self.joint_stamp < .3
            if base_fresh:
                delta = self.native_pose[:2]-state[:2]
                c, s = np.cos(state[2]), np.sin(state[2])
                command[0] = self.native_velocity@np.array([c, s])+delta@np.array([c, s])
                yaw_error = (self.native_pose[2]-state[2]+np.pi)%(2*np.pi)-np.pi
                command[1] = 1.5*yaw_error+.8*(delta@np.array([-s, c]))
            if arm_fresh:
                command[2:] = self.native_dq+1.8*(self.native_q-state[5:11])-.2*state[11:17]
            return command, base_fresh or arm_fresh

    def integrate(self, dt):
        if dt <= 0:
            return
        count = max(1, int(math.ceil(dt/.005)))
        step = dt/count
        for index in range(count):
            subtime = self.t-dt+(index+1)*step
            command, fresh = self.controls(subtime)
            if self.last_command_fresh and not fresh:
                self.command_expiry_transitions += 1
            self.last_command_fresh = fresh
            elapsed = -1. if self.motion_time is None else subtime-self.motion_time
            self.plant.step(command, step, elapsed, subtime)
        self.s = self.plant.state[:5].copy()
        self.q = self.plant.state[5:11].copy()
        self.dq = self.plant.state[11:17].copy()
        self.min_clearance = self.plant.min_external_gap
        self.collided = self.plant.collided
        if self.group == 'native':
            self.whole_command = command.copy()
            self.command = command[:2].copy()

    def cloud(self, stamp):
        # Activation is tied to the first actual synthetic cloud, after this
        # loop's integration. Earlier substeps are correctly audited box-free.
        if (self.box_runtime is None and self.motion_time is not None
                and self.t-self.motion_time >= self.scene_spec['appearance_s']):
            runtime = place_box(self.model, self.q, self.s[:3], self.scene_spec)
            runtime.update(appearance_time=self.t, appearance_ros_time=1000+self.t,
                motion_anchor=self.motion_time,
                scheduled_appearance_time=self.motion_time+self.scene_spec['appearance_s'],
                activation='first cloud generation; no earlier audit occupancy')
            self.box_runtime = runtime
            self.plant.box = runtime['box']
            self.plant.audit(self.t-self.motion_time, self.t)
            (self.out/'box_runtime.json').write_text(json.dumps(runtime, indent=2)+'\n')
            self.events.append(dict(t=self.t, kind='box_appearance', **runtime))
        world = np.concatenate([self.static, obstacle_points(self.plant.box)])
        delta = world-np.array([self.s[0], self.s[1], 0.])
        c, s = math.cos(self.s[2]), math.sin(self.s[2])
        rotation = np.array([[c, s, 0.], [-s, c, 0.], [0., 0., 1.]])
        local = delta@rotation.T-self.offset
        local = local[np.linalg.norm(local[:, :2], axis=1) < 5.]
        self.raw_pub.publish(pc2.create_cloud_xyz32(Header(stamp=stamp, frame_id='livox_frame'), local))

    def record(self):
        self.rows.append([self.t, *self.s, *self.command, self.limit,
            self.plant.external_gap, self.traj_count, self.dynamic_n, int(self.health),
            *self.progress, 0., *self.q, *self.dq, *self.whole_command[2:],
            self.plant.configuration_gap, int(self.last_command_fresh),
            self.plant.base_box_gap, self.plant.arm_box_gap])
        if self.plant.box is not None:
            box = self.plant.box
            self.obstacle_rows.append([self.t, *box['center'], *box['size'], box['id']])

    def write_results(self, reason):
        # Retain the established result metrics and exact state column prefix.
        header = ('t,x,y,yaw,v,w,cmd_v,cmd_w,safety_limit,clearance,trajectory_count,dynamic_points,'
            'perception_healthy,trajectory_start,virtual_time,time_scale,arm_error').split(',')
        header += ['q'+str(i+1) for i in range(6)]+['dq'+str(i+1) for i in range(6)]
        header += ['cmd_dq'+str(i+1) for i in range(6)]
        header += ['configuration_gap', 'whole_body_command_fresh', 'base_box_gap', 'arm_box_gap']
        with (self.out/'state.csv').open('w') as stream:
            writer = csv.writer(stream)
            writer.writerow(header)
            writer.writerows(self.rows)
        with (self.out/'obstacles.csv').open('w') as stream:
            writer = csv.writer(stream)
            writer.writerow('t,cx,cy,cz,dx,dy,dz,id'.split(','))
            writer.writerows(self.obstacle_rows)
        for name, value in [('events', self.events), ('trajectories', self.raw_trajs),
                            ('mppi_diagnostics', self.diagnostics)]:
            (self.out/(name+'.json')).write_text(json.dumps(value, indent=2)+'\n')
        result = dict(reason=reason, goal_reached=reason == 'goal', goal_sent=self.goal_time,
            motion_anchor=self.motion_time, scenario_start='motion', trajectory_bundles=self.traj_count,
            min_clearance=self.plant.min_external_gap, min_external_gap_m=self.plant.min_external_gap,
            min_configuration_gap_m=self.plant.min_configuration_gap,
            min_base_box_gap_m=self.plant.min_base_box_gap, min_arm_box_gap_m=self.plant.min_arm_box_gap,
            collision_physical=self.plant.collided, joint_limit_violation=self.plant.joint_violation,
            first_violation=self.plant.first_violation, final_state=self.s.tolist(),
            final_q=self.q.tolist(), final_dq=self.dq.tolist(), elapsed=self.t, scene=self.scene,
            arm_max_displacement_rad=self.plant.arm_max_displacement.tolist(),
            arm_total_motion_rad=self.plant.arm_motion.tolist(),
            command_accepted=self.command_accepted, command_rejected=self.command_rejected,
            command_expiry_transitions=self.command_expiry_transitions,
            native_arm_commands_ignored=self.native_arm_commands_ignored,
            native_base_commands_received=self.native_base_count,
            native_arm_commands_received=self.native_arm_count,
            native_commands_received=self.native_base_count+self.native_arm_count,
            physical_base_locked=False, box_runtime=self.box_runtime,
            maximum_integration_step_s=self.plant.max_substep, audit_substeps=self.plant.substeps,
            collision_pair_policy=self.model.collision_policy(),
            native_adapter='same standby pose/vxy and q/dq velocity adapter; shared bounded plant',
            scope='Native numerical ROS plant; exact sphere/AABB sampled audit; no mesh/continuous/hardware certification')
        (self.out/'result.json').write_text(json.dumps(result, indent=2)+'\n')
        if self.box_runtime is not None:
            # Run this sampled geometric witness after execution so it cannot
            # introduce an artificial sensor/control pause at appearance.
            precheck = straight_path_precheck(self.model, self.scene_spec, self.box_runtime)
            (self.out/'runtime_geometry_precheck.json').write_text(json.dumps(precheck, indent=2)+'\n')


if __name__ == '__main__':
    if not ROS_AVAILABLE:
        raise RuntimeError('Source the isolated ROS workspace first')
    isolation()
    rospy.init_node('arm_box_simulator')
    if not rospy.get_param('/online_validation/enabled', False) or not rospy.get_param('/use_sim_time', False):
        raise RuntimeError('Missing simulation interlock')
    ArmBoxSimulator().run()
