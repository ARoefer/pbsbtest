#!/usr/bin/env python

import rospy
import sys
import tf
import symengine as sp

from time import time
from blessed import Terminal
from collections import OrderedDict

from sensor_msgs.msg import JointState as JointStateMsg
from std_msgs.msg import Empty as EmptyMsg
from rosgraph_msgs.msg import Clock as ClockMsg
from geometry_msgs.msg import WrenchStamped as WrenchMsg
from trajectory_msgs.msg import JointTrajectory as JointTrajectoryMsg
from trajectory_msgs.msg import JointTrajectoryPoint as JointTrajectoryPointMsg

from gebsyas.ros_visualizer import ROSVisualizer
from giskardpy.input_system import FrameInput, JointStatesInput
from giskardpy.symengine_controller import SymEngineController as Controller
from giskardpy.symengine_controller import position_conv, rotation_conv
from giskardpy.symengine_wrappers import *
from giskardpy.god_map import GodMap
from giskardpy.trajectory import SingleJointState
from giskardpy.utils import dict_to_joint_states
from giskardpy.qp_problem_builder import SoftConstraint

from iai_bullet_sim.basic_simulator import BasicSimulator
from iai_bullet_sim.utils import Frame, Vector3, Quaternion
from iai_bullet_sim.ros_modules import JSPublisher, SensorPublisher

from pbsbtest.utils import res_pkg_path

def run_controller_until(god_map, controller, initial_js, js_prefix, term_goal_list, max_iterations=500):
    """
    Runs a controller until a set of goal terms t_g satisfy t_g < v_g, or a maximum number of iterations is reached.
    Returns the final joint configuration and the positional trajectory generated during the iterations.
    Raises an error, if the maximum number of iterations is reached without satisfying all goals.

    :param controller:
    :type controller: Controller
    :param term_goal_list:
    :type term_goal_list: [tuple]
    :param max_iterations:
    :type max_iterations:
    :return: (final_js, trajectory)
    :rtype:
    """
    js = {j: SingleJointState(j, s.position, s.velocity, s.effort) for j, s in initial_js.items()}
    trajectory = OrderedDict()
    traj_stamp = rospy.Duration(0.0)
    ros_deltaT = rospy.Duration(deltaT)
    it = 0
    god_map.set_data([js_prefix], js)
    while True and not rospy.is_shutdown():
        trajectory[traj_stamp] = {j: SingleJointState(j, s.position, s.velocity, s.effort) for j, s in js.items()}
        cvals = god_map.get_expr_values()
        if min([goal_expr.subs(cvals) < goal_val for goal_expr, goal_val in term_goal_list]) is True:
            break
        if it >= max_iterations:
            raise Exception('Failed to converge towards solution within {} iterations.'.format(max_iterations))
        it  +=1
        for j, v in controller.get_cmd(cvals).items():
            js[j].position += v * deltaT
        traj_stamp += ros_deltaT

    return (js, trajectory)


pi = 3.14159265359

class Node(object):
    def __init__(self, urdf_path, base_link, eef_link, wrist_link, wps, projection_frame):
        self.vis = ROSVisualizer('force_test', base_frame=base_link, plot_topic='plot')
        self.wps = [Frame(Vector3(*wp[3:]), Quaternion(*list(quaternion_from_rpy(*wp[:3])))) for wp in wps]
        self.wpsIdx = 0
        self.base_link = base_link
        self.god_map = GodMap()

        self.js_prefix = 'joint_state'

        self.js_pub = rospy.Publisher('/joint_states', JointStateMsg, queue_size=1, tcp_nodelay=True)
        self.traj_pub = rospy.Publisher('/ur5_table_description/commands/joint_trajectory', JointTrajectoryMsg, queue_size=1, tcp_nodelay=True)
        self.wrench_pub = rospy.Publisher('wrist_force_transformed', WrenchMsg, queue_size=1, tcp_nodelay=True)

        # Giskard ----------------------------
        # Init controller, setup controlled joints
        self.controller = Controller(res_pkg_path(urdf_path), res_pkg_path('package://pbsbtest/.controllers/'), 0.6)
        self.robot = self.controller.robot

        self.js_input = JointStatesInput.prefix_constructor(self.god_map.get_expr, self.robot.get_joint_names(), self.js_prefix, 'position')

        self.robot.set_joint_symbol_map(self.js_input)
        self.controller.set_controlled_joints(self.robot.get_joint_names())

        # Get eef and sensor frame
        self.sensor_frame = self.robot.get_fk_expression(base_link, wrist_link)
        self.eef_frame    = self.robot.get_fk_expression(base_link, eef_link)
        self.eef_pos      = pos_of(self.eef_frame)

        # Construct motion frame
        mplate_pos = pos_of(self.robot.get_fk_expression(base_link, 'arm_mounting_plate'))
        self.motion_frame = frame3_rpy(pi, 0, pi * 0.5, mplate_pos)

        wp_frame = self.motion_frame * FrameInput.prefix_constructor('goal/position', 'goal/quaternion', self.god_map.get_expr).get_frame()
        wp_pos = pos_of(wp_frame)


        # Projection frame
        self.world_to_projection_frame = projection_frame

        # Pre init
        self.god_map.set_data(['goal'], self.wps[0])
        self.tick_rate = 50
        deltaT = 1.0 / self.tick_rate
        js = {j: SingleJointState(j) for j in self.robot.get_joint_names()}
        self.god_map.set_data([self.js_prefix], js)

        err = norm(wp_pos - self.eef_pos)
        rot_err = dot(wp_frame * unitZ, self.eef_frame * unitZ)

        scons = position_conv(wp_pos, self.eef_pos) # {'align position': SoftConstraint(-err, -err, 1, err)}
        scons.update(rotation_conv(rot_of(wp_frame), rot_of(self.eef_frame), rot_of(self.eef_frame).subs(self.god_map.get_expr_values())))
        self.controller.init(scons, self.god_map.get_free_symbols())


        print('Solving for initial state...')
        # Go to initial configuration
        js = run_controller_until(self.god_map, self.controller, js, self.js_prefix, [(err, 0.01), (1 - rot_err, 0.001)])[0]

        print('About to plan trajectory...')

        # Generate trajectory
        trajectory = OrderedDict()
        traj_stamp = rospy.Duration(0.0)
        section_stamps = []
        for x in range(1, len(self.wps)):
            print('Heading for point {}'.format(x))
            self.god_map.set_data(['goal'], self.wps[x])
            js, sub_traj = run_controller_until(self.god_map, self.controller, js, self.js_prefix, [(err, 0.01)])
            for time_code, position in sub_traj.items():
                trajectory[traj_stamp + time_code] = position
            traj_stamp = sub_traj.keys()[-1]


        print('Trajectory planned! {} points over a duration of {} seconds.'.format(len(trajectory), len(trajectory) * deltaT))


        traj_msg = JointTrajectoryMsg()
        traj_msg.header.stamp = rospy.Time.now()
        traj_msg.joint_names  = trajectory.items()[0][1].keys()
        for t, v in trajectory.items():
            point = JointTrajectoryPointMsg()
            point.positions = [v[j].position for j in traj_msg.joint_names]
            point.time_from_start = t
            traj_msg.points.append(point)


        raw_input('Press ENTER to publish trajectory.')
        self.traj_pub.publish(traj_msg)
        self.tfListener = tf.TransformListener()
        self.wrench_sub = rospy.Subscriber('/ur5_table_description/sensors/wrist_3_joint', WrenchMsg, callback=self.transform_wrench, queue_size=1)


    def transform_wrench(self, wrench_msg):
        try:
            self.tfListener.waitForTransform(self.base_link, wrench_msg.header.frame_id, rospy.Time(0), rospy.Duration(0.1))
            (trans,rot) = self.tfListener.lookupTransform(self.base_link, wrench_msg.header.frame_id, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print('Failed to transform wrench from {} to {}'.format(wrench_msg.header.frame_id, self.base_link))
            return

        f = wrench_msg.wrench.force

        f = self.world_to_projection_frame * frame3_quaternion(trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3]) * vector3(f.x, f.y, f.z)
        #t = self.projection_frame.subs(cvals) * vec3(*state.m)

        sensor_msg = WrenchMsg()
        sensor_msg.header.stamp = wrench_msg.header.stamp
        sensor_msg.wrench.force.x  = f[0]
        sensor_msg.wrench.force.y  = f[1]
        sensor_msg.wrench.force.z  = f[2]
        # sensor_msg.wrench.torque.x = t[0]
        # sensor_msg.wrench.torque.y = t[1]
        # sensor_msg.wrench.torque.z = t[2]
        self.wrench_pub.publish(sensor_msg)


if __name__ == '__main__':
    rospy.init_node('force_test')

    urdf_path = 'package://iai_table_robot_description/robots/ur5_table.urdf'

    wrist_link = 'ee_link'
    eef_frame = 'gripper_tool_frame'#
    base_link = 'map'


    wps = [(0, 0, 0, -0.1, 0.5, -0.1),
           (0, 0, 0, -0.1, 0.5,  0.01),
           (0, 0, 0,  0.1, 0.5,  0.01),
           (0, 0, 0,  0.1, 0.5, -0.1)] #[point3(1,1,1.1), point3(1,1,1), point3(0.2,0,1), point3(0.2,0,1.1)]

    twp = point3(*wps[2][3:])
    pwp = point3(*wps[1][3:])
    x_v = unitY# twp - pwp
    #x_v = x_v / norm(x_v) # Normalize x_v
    z_v = vector3(0,0,-1)
    y_v = cross(z_v, x_v)

    proj_frame = Matrix([[x_v[0], x_v[1], x_v[2], 0],
                         [y_v[0], y_v[1], y_v[2], 0],
                         [z_v[0], z_v[1], z_v[2], 0],
                         [     0,      0,      0, 1]])

    node = Node(urdf_path, base_link, eef_frame, wrist_link, wps, proj_frame)

    while not rospy.is_shutdown():
        pass