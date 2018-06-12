#!/usr/bin/env python

import rospy
from gebsyas.ros_visualizer import ROSVisualizer
from giskardpy.robot import Robot
from giskardpy.symengine_wrappers import point3, pos_of, norm, x_col, y_col, z_col, unitZ, dot, frame3_rpy, abs
from pbsbtest.utils import res_pkg_path
from sensor_msgs.msg import JointState as JointStateMsg
from std_msgs.msg import Empty as EmptyMsg

from giskardpy import USE_SYMENGINE
from giskardpy.qpcontroller import QPController
from giskardpy.qp_problem_builder import SoftConstraint
from giskardpy.input_system import ControllerInputArray, ScalarInput
from giskardpy.input_system import FrameInputRPY as FrameInput


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
        eef_forward = x_col(eef_frame)


        goal_expr = self.goal_eef[self.eef].get_expression()
        goal_x = x_col(goal_expr)
        goal_y = y_col(goal_expr)
        goal_z = z_col(goal_expr)
        
        x_z_dot = dot(eef_forward, -goal_z)
        x_z_dot_ctrl = 1 - x_z_dot
        y_x_dot = dot(eef_y, goal_x)
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
    def __init__(self, urdf_path, base_link, eef_frame, wrist_link, wps, pub_topic, js_topic):
        self.wps = wps
        self.wpsIdx = 0
        
        self.vis = ROSVisualizer('force_test', base_frame=base_link, plot_topic='plot')

        self.robot = Robot()
        self.robot.load_from_urdf_path(res_pkg_path(urdf_path), base_link, [eef_frame])
        self.wrist_pos = pos_of(self.robot.frames[wrist_link])

        self.controller = EEFPositionControl(self.robot, eef_frame, wrist_link, velocity=1)
        self.controller.set_goal(self.wps[self.wpsIdx])

        self.cmd_pub  = rospy.Publisher(pub_topic, JointStateMsg, queue_size=1)
        self.tick_pub = rospy.Publisher('/trigger_tick', EmptyMsg, queue_size=1)
        self.js_sub  = rospy.Subscriber(js_topic, JointStateMsg, callback=self.js_callback, queue_size=1)        

    def js_callback(self, js_msg):
        js = {js_msg.name[x]: js_msg.position[x] for x in range(len(js_msg.name))}
        self.robot.set_joint_state(js)
        eef_position = self.wrist_pos.subs(js)
        dist = norm(eef_position - point3(*(self.wps[self.wpsIdx][3:])))        
        if dist < 0.03:
            self.wpsIdx = (self.wpsIdx + 1) % len(wps)
            print('Waypoint reaced. Next one is number {}'.format(self.wpsIdx))
            self.controller.set_goal(self.wps[self.wpsIdx])

        next_cmd = self.controller.get_next_command()
        cmd_msg = JointStateMsg()
        cmd_msg.header.stamp = rospy.Time.now()
        for jname, vc in next_cmd.items():
            cmd_msg.name.append(jname)
            cmd_msg.velocity.append(vc)

        self.cmd_pub.publish(cmd_msg)

        self.vis.begin_draw_cycle()
        self.vis.draw_sphere('eef_giskard', eef_position, 0.025)
        self.vis.draw_sphere('goal', self.wps[self.wpsIdx][3:], 0.025, r=0, g=1)
        self.vis.render()

        emsg =  EmptyMsg()
        self.tick_pub.publish(emsg)


if __name__ == '__main__':
    rospy.init_node('force_test')
    
    urdf_path = 'package://iai_table_robot_description/robots/ur5_table.urdf'

    wrist_link = 'wrist_3_link'
    eef_frame = 'ee_link'#
    base_link = 'map'


    on_table_height = 0.9

    wps = [(0,0,0, -0.3,-0.4,on_table_height + 0.1),
           (0,0,0, -0.3,-0.4,on_table_height), 
           (0,0,0, -0.3,-0.2,on_table_height),
           (0,0,0, -0.3,-0.2,on_table_height + 0.1)] #[point3(1,1,1.1), point3(1,1,1), point3(0.2,0,1), point3(0.2,0,1.1)]

    node = Node(urdf_path, base_link, eef_frame, wrist_link, wps, 
                '/ur5_table_description/commands/joint_velocities',
                '/ur5_table_description/joint_state')

    while not rospy.is_shutdown():
        pass