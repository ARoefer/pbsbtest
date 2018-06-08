#!/usr/bin/env python

import rospy
from gebsyas.simulator import BulletSimulator, vec3_to_list
from gebsyas.ros_visualizer import ROSVisualizer
import pybullet as pb
from giskardpy.robot import Robot
from giskardpy.symengine_wrappers import point3, pos_of, norm, x_col, y_col, z_col, unitZ, dot, frame3_rpy, abs
from pbsbtest.utils import res_pkg_path
from sensor_msgs.msg import JointState as JointStateMsg

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
        eef_forward = z_col(eef_frame)


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


if __name__ == '__main__':
    rospy.init_node('force_test')
    
    sim = BulletSimulator(50)

    sim.init(mode=pb.GUI)

    urdf_path = 'package://fetch_description/robots/fetch.urdf' # 'package://ur_description/urdf/ur5_robot.urdf'  # 'package://iai_table_robot_description/robots/ur5_table.urdf'

    robotId = sim.load_robot(urdf_path)
    # initial_js = {"elbow_joint": 0,
    #               "shoulder_lift_joint": 0,
    #               "shoulder_pan_joint": 0,
    #               "wrist_1_joint": 0,
    #               "wrist_2_joint": 0,
    #               "wrist_3_joint": 0,
    #               "gripper_base_gripper_left_joint": -0.04,
    #               "gripper_joint": 0.08}
    
    #print('\n'.join(['  {}: {}'.format(j, p) for j, p in initial_js.items()]))

    #sim.set_joint_positions(robotId, initial_js)

    wrist_link = 'wrist_roll_link' # 'wrist_3_link'
    eef_frame = 'gripper_link' # 'gripper_tool_frame' # 'ee_link'#
    base_link = 'base_link' # 'map'

    robot = Robot()
    robot.load_from_urdf_path(res_pkg_path(urdf_path), base_link, [eef_frame]) # 'base_link'

    controller = EEFPositionControl(robot, eef_frame, wrist_link, velocity=0.05)

    on_table_height = 0.7

    wps = [(0,0,0, -0.3,-0.4,on_table_height + 0.1),
           (0,0,0, -0.3,-0.4,on_table_height), 
           (0,0,0, -0.3,-0.2,on_table_height),
           (0,0,0, -0.3,-0.2,on_table_height + 0.1)] #[point3(1,1,1.1), point3(1,1,1), point3(0.2,0,1), point3(0.2,0,1.1)]
    wpsIdx = 0
    controller.set_goal(wps[wpsIdx])


    eef_state = sim.get_link_state(robotId, eef_frame)

    connectorRbId = pb.createRigidBody(pb.GEOM_SPHERE, radius=0.005, position=eef_state.CoMFrame.position, mass=0.001)
    
    robotBId = sim.bodies[robotId].bulletId
    eef_BIdx = sim.bodies[robotId].link_index_map[eef_frame]
    connectorJId  = pb.createConstraint(robotBId, eef_BIdx, connectorRbId, -1, pb.JOINT_FIXED,[0,0,0],[0,0,0],[0,0,0],[0,0,0,1],[0,0,0,1])

    vis = ROSVisualizer('force_test/visual', base_link)

    sim_pub = rospy.Publisher('/joint_states', JointStateMsg, queue_size=5)

    while True:
        js = {jn: j.position for jn, j in sim.get_joint_state(robotId).items()}
        robot.set_joint_state(js)
        vis.begin_draw_cycle()
        #vis.draw_robot_pose('robot', robot, js)
        next_cmd = controller.get_next_command()
        js_msg = JointStateMsg()
        js_msg.header.stamp = rospy.Time.now()

        for j, p in js.items():
            js_msg.name.append(j)
            js_msg.velocity.append(0)
            js_msg.position.append(p)
            js_msg.effort.append(0)

        sim_pub.publish(js_msg)

        #print('\n'.join(['  {}: {}'.format(j, p) for j, p in next_cmd.items()]))
        sim.apply_joint_vel_cmds(robotId, next_cmd)
        sim.update()

        b_eef_pos = pos_of(sim.get_link_state_sym(robotId, wrist_link).CoMFrame)
        vis.draw_sphere('eef_bullet', b_eef_pos, 0.025)
        vis.draw_sphere('goal', wps[wpsIdx][3:], 0.025, r=0, g=1)
        vis.render()

        dist = norm(b_eef_pos - point3(*(wps[wpsIdx][3:])))
        #print('Distance to goal: {}'.format(dist))
        if dist < 0.03:
            wpsIdx = (wpsIdx + 1) % len(wps)
            print('Waypoint reaced. Next one is number {}'.format(wpsIdx))
            controller.set_goal(wps[wpsIdx])
            print('Joint state:\n{}'.format(str(js)))