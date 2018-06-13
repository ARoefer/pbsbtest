#!/usr/bin/env python

import rospy
import sys
from time import time
from gebsyas.ros_visualizer import ROSVisualizer
from giskardpy.robot import Robot
from giskardpy.symengine_wrappers import point3, vec3, pos_of, norm, x_col, y_col, z_col, unitZ, dot, frame3_rpy, abs, pi
from pbsbtest.utils import res_pkg_path
from sensor_msgs.msg import JointState as JointStateMsg
from std_msgs.msg import Empty as EmptyMsg
from rosgraph_msgs.msg import Clock as ClockMsg

from giskardpy import USE_SYMENGINE
from giskardpy.qpcontroller import QPController
from giskardpy.qp_problem_builder import SoftConstraint
from giskardpy.input_system import ControllerInputArray, ScalarInput
from giskardpy.input_system import FrameInputRPY as FrameInput
from geometry_msgs.msg import WrenchStamped as WrenchMsg

from iai_bullet_sim.basic_simulator import BasicSimulator
from iai_bullet_sim.ros_modules import JSPublisher, SensorPublisher

from blessed import Terminal

class EEFPositionControl(QPController):
    def __init__(self, robot, eef, wrist, velocity=0.2, weight=1):
        self.weight = weight
        self.velocity = velocity
        self.eef = eef
        self.wrist = wrist
        super(EEFPositionControl, self).__init__(robot)

    # @profile
    def add_inputs(self, robot):
        self.goal_eef = {}
        self.goal_weights = {}
        for eef in robot.end_effectors:
            self.goal_eef[eef] = FrameInput(prefix=eef, suffix='goal')
            self.goal_weights[eef] = ScalarInput(prefix=eef, suffix='sc_w')

    # @profile
    def make_constraints(self, robot):
        wrist_frame = robot.frames[self.wrist]
        eef_frame = robot.frames[self.eef]
        eef_y = y_col(eef_frame)
        eef_forward = z_col(eef_frame)


        goal_expr = self.goal_eef[self.eef].get_expression()
        goal_x = x_col(goal_expr)
        goal_y = y_col(goal_expr)
        goal_z = z_col(goal_expr)
        
        x_z_dot = dot(eef_forward, -goal_z)
        x_z_dot_ctrl = 1 - x_z_dot
        y_x_dot = abs(dot(eef_y, goal_x))
        y_x_dot_ctrl = 1 - y_x_dot
        # dist = norm(sp.Add(pos_of(eef_frame), - goal_expr, evaluate=False))
        eef_in_goal = pos_of(wrist_frame) - pos_of(goal_expr)
        dist_x = abs(dot(goal_x, eef_in_goal))
        dist_y = abs(dot(goal_y, eef_in_goal))
        dist_z = abs(dot(goal_z, eef_in_goal))
        #dist_ctrl = x_z_dot * y_x_dot * -min(dist, self.velocity)
        
        self._soft_constraints['align {} position_x'.format(self.eef)] = SoftConstraint(lower=-dist_x,
                                                                        upper=-dist_x,
                                                                        weight=self.goal_weights[self.eef].get_expression(),
                                                                        expression=dist_x)
        self._soft_constraints['align {} position_y'.format(self.eef)] = SoftConstraint(lower=-dist_y,
                                                                        upper=-dist_y,
                                                                        weight=self.goal_weights[self.eef].get_expression(),
                                                                        expression=dist_y)
        self._soft_constraints['align {} position_z'.format(self.eef)] = SoftConstraint(lower=-dist_z,
                                                                        upper=-dist_z,
                                                                        weight=self.goal_weights[self.eef].get_expression(),
                                                                        expression=dist_z)

        self._soft_constraints['align {}.z to -goal.z'.format(self.eef)] = SoftConstraint(x_z_dot_ctrl, 
                                                                                        x_z_dot_ctrl, 
                                                                                        self.goal_weights[self.eef].get_expression(),
                                                                                        x_z_dot)
        self._soft_constraints['align {}.y to goal.y'.format(self.eef)] = SoftConstraint(y_x_dot_ctrl, 
                                                                                    y_x_dot_ctrl, 
                                                                                    self.goal_weights[self.eef].get_expression(),
                                                                                    y_x_dot)
        self._controllable_constraints = robot.joint_constraints
        self._hard_constraints = robot.hard_constraints
        self.update_observables({self.goal_weights[self.eef].get_symbol_str(): self.weight})
        self.set_goal([0]*6)


    def set_goal(self, goal):
        """
        dict eef_name -> goal_position
        :param goal_pos:
        :return:
        """
        #for eef, goal_pos in goal.items():
        self.update_observables(self.goal_eef[self.eef].get_update_dict(*goal))


class Node(object):
    def __init__(self, urdf_path, base_link, eef_frame, wrist_link, wps):
        self.vis = ROSVisualizer('force_test', base_frame=base_link, plot_topic='plot')
        self.wps = wps
        self.wpsIdx = 0

        # Giskard
        self.robot = Robot()
        self.robot.load_from_urdf_path(res_pkg_path(urdf_path), base_link, [eef_frame])
        self.sensor_frame = self.robot.frames[wrist_link]
        self.wrist_pos = pos_of(self.sensor_frame)


        mplate_pos = pos_of(self.robot.frames['arm_mounting_plate'])
        self.motion_frame = frame3_rpy(pi, 0, pi * 0.5, mplate_pos)
        self.sensor_motion_frame_inv = self.motion_frame.inv() * self.sensor_frame

        self.controller = EEFPositionControl(self.robot, eef_frame, wrist_link, velocity=1)
        waypoint = self.motion_frame * point3(*(self.wps[self.wpsIdx][3:]))
        self.controller.set_goal((0,0,0, waypoint[0], waypoint[1], waypoint[2]))


        # Sim things
        self.tick_rate = 50
        self.sim = BasicSimulator(self.tick_rate)
        self.sim.init(mode='gui')
    
        plane = self.sim.load_urdf('package://iai_bullet_sim/urdf/plane.urdf', useFixedBase=1)

        self.sensor_joint = 'wrist_3_joint'
        self.ur5 = self.sim.load_urdf(urdf_path, useFixedBase=1)
        self.ur5.enable_joint_sensor(self.sensor_joint)
        ur5Id = self.sim.get_body_id(self.ur5.bId())
        self.ur5_js_pub = JSPublisher(self.ur5, ur5Id)
        self.sim.register_post_physics_cb(self.ur5_js_pub.update)
        self.sensor_pub = rospy.Publisher('{}/sensors/{}'.format(ur5Id, self.sensor_joint), WrenchMsg, queue_size=1)
        
        self.clock_pub = rospy.Publisher('/clock', ClockMsg, queue_size=1)
        self.time = rospy.Time()
        self.time_step = rospy.Duration(1.0 / self.tick_rate)
        self.terminal = Terminal()
        self.__last_tick = None
        self.__last_msg_len = 0

        self.ur5_fixed_positions = {#'gripper_base_gripper_left_joint': 0,
                                    'gripper_joint': 0}


    def tick(self):
        self.time += self.time_step
        cmsg = ClockMsg()
        cmsg.clock = self.time
        self.clock_pub.publish(cmsg)

        js = {j: s.position for j, s in self.ur5.joint_state().items()}
        self.robot.set_joint_state(js)
        eef_position = self.wrist_pos.subs(js)
        waypoint = self.motion_frame * point3(*(self.wps[self.wpsIdx][3:]))
        dist = norm(eef_position - waypoint)        
        if dist < 0.03:
            self.wpsIdx = (self.wpsIdx + 1) % len(wps)
            print('\nWaypoint reaced. Next one is number {}'.format(self.wpsIdx))
        
        self.controller.set_goal((0,0,0, waypoint[0], waypoint[1], waypoint[2]))

        next_cmd = self.controller.get_next_command()


        self.ur5.apply_joint_vel_cmds(next_cmd)
        self.ur5.apply_joint_pos_cmds(self.ur5_fixed_positions)

        self.sim.update()

        state = self.ur5.get_sensor_states()['wrist_3_joint']
        f = self.sensor_motion_frame_inv.subs(js) * vec3(*state.f)
        t = self.sensor_motion_frame_inv.subs(js) * vec3(*state.m) 

        sensor_msg = WrenchMsg()
        sensor_msg.header.stamp = rospy.Time.now()
        sensor_msg.wrench.force.x  = f[0] 
        sensor_msg.wrench.force.y  = f[1] 
        sensor_msg.wrench.force.z  = f[2] 
        sensor_msg.wrench.torque.x = t[0] 
        sensor_msg.wrench.torque.y = t[1] 
        sensor_msg.wrench.torque.z = t[2] 
        self.sensor_pub.publish(sensor_msg)

        self.vis.begin_draw_cycle()
        self.vis.draw_sphere('eef_giskard', eef_position, 0.025)
        self.vis.draw_sphere('goal', waypoint, 0.025, r=0, g=1)
        self.vis.render()

        now = time()
        if self.__last_tick != None:
            deltaT = now - self.__last_tick
            ratio  = self.time_step.to_sec() / deltaT
            sys.stdout.write(self.terminal.move(self.terminal.height,  0))
            msg = u'Simulation Factor: {:03.4f}'.format(ratio)
            self.__last_msg_len = len(msg)
            sys.stdout.write(msg)
            sys.stdout.flush()
        self.__last_tick = now


if __name__ == '__main__':
    rospy.init_node('force_test')
    
    urdf_path = 'package://iai_table_robot_description/robots/ur5_table.urdf'

    wrist_link = 'ee_link'
    eef_frame = 'gripper_tool_frame'#
    base_link = 'map'


    wps = [(0,0,0, -0.1, 0.5, -0.1 - 0.213),
           (0,0,0, -0.1, 0.5, -0.213), 
           (0,0,0,  0.1, 0.5, -0.213),
           (0,0,0,  0.1, 0.5, -0.1 - 0.213)] #[point3(1,1,1.1), point3(1,1,1), point3(0.2,0,1), point3(0.2,0,1.1)]

    node = Node(urdf_path, base_link, eef_frame, wrist_link, wps)

    while not rospy.is_shutdown():
        node.tick()