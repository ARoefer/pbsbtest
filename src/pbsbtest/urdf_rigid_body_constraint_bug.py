#!/usr/bin/env python
import pybullet as pb
from pbsbtest.utils import res_pkg_path
from collections import namedtuple

VectorTuple = namedtuple('VectorTuple', ['x', 'y', 'z'])
QuatT = namedtuple('QuatT', ['x', 'y', 'z', 'w'])
FrameTuple  = namedtuple('FrameTuple', ['position', 'quaterion'])

JointInfo = namedtuple('JointInfo', ['jointIndex', 'jointName', 'jointType', 'qIndex', 'uIndex',
									 'flags', 'jointDamping', 'jointFriction', 'jointLowerLimit',
									 'jointUpperLimit', 'jointMaxForce', 'jointMaxVelocity', 'linkName',
									 'jointAxis', 'parentFramePos', 'parentFrameOrn', 'parentIndex'])
LinkState  = namedtuple('LinkState', ['CoMFrame', 'localInertialFrame', 'worldFrame', 'linearVelocity', 'angularVelocity'])

def vec3_to_list(vec):
	return [vec[0], vec[1], vec[2]]



if __name__ == '__main__':
	physicsClient = pb.connect(pb.GUI, options="--opengl2")#or p.DIRECT for non-graphical version
	pb.setGravity(0,0, -9.81)
	planeId =  pb.loadURDF(res_pkg_path("package://pbsbtest/urdf/plane.urdf"))
	robotId = pb.loadURDF(res_pkg_path("package://pbsbtest/robots/fetch.urdf"), useFixedBase=1)

	zero_vector = VectorTuple(0,0,0)

	joint_map = {}
	joint_idx_map = []
	for x in range(pb.getNumJoints(robotId)):
		joint = JointInfo(*pb.getJointInfo(robotId, x))
		joint_idx_map.append(joint.jointName)
		joint_map[joint.jointName] = joint

	link_index_map = {info.linkName: info.jointIndex for info in joint_map.values()}

	ls = pb.getLinkState(robotId, link_index_map['gripper_link'], 0)
	ls = LinkState(FrameTuple(VectorTuple(*ls[0]), QuatT(*ls[1])),
							  FrameTuple(VectorTuple(*ls[2]), QuatT(*ls[3])),
							  FrameTuple(VectorTuple(*ls[4]), QuatT(*ls[5])),
							  zero_vector,
							  zero_vector)

	geomTypes = [pb.GEOM_BOX, pb.GEOM_SPHERE, pb.GEOM_CAPSULE, pb.GEOM_CYLINDER]
	
	connectorId  = pb.createRigidBody(pb.GEOM_SPHERE, radius=0.05, position=[1,0,1])

	cId = pb.createConstraint(robotId, link_index_map['gripper_link'], 
							  connectorId, -1, 
							  pb.JOINT_FIXED, [0,0,0], 
							  [0,0.2,0], [1,0,1], [0,0,0,1], [0,0,0,1])

	while True:
		if raw_input('TYPE Q TO END\n') in ['q', 'Q']:
			break

		pb.stepSimulation()


	pb.disconnect()