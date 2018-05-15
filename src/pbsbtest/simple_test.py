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
	# robotId = pb.loadURDF(res_pkg_path("package://fetch_description/robots/fetch.urdf"), useFixedBase=1)

	zero_vector = VectorTuple(0,0,0)

	joint_map = {}
	joint_idx_map = []
	# for x in range(pb.getNumJoints(robotId)):
	# 	joint = JointInfo(*pb.getJointInfo(robotId, x))
	# 	joint_idx_map.append(joint.jointName)
	# 	joint_map[joint.jointName] = joint

	# link_index_map = {info.linkName: info.jointIndex for info in joint_map.values()}

	# # ls = pb.getLinkState(robotId, link_index_map['gripper_link'], 0)
	# # ls = LinkState(FrameTuple(VectorTuple(*ls[0]), QuatT(*ls[1])),
	# # 						  FrameTuple(VectorTuple(*ls[2]), QuatT(*ls[3])),
	# # 						  FrameTuple(VectorTuple(*ls[4]), QuatT(*ls[5])),
	# # 						  zero_vector,
	# # 						  zero_vector)


	sbId = pb.loadSoftBody(res_pkg_path('package://pbsbtest/meshes/weird_sponge.obj'), 0.5, 1, 0.04)
	pb.resetBasePositionAndOrientation(sbId, [0,0,1.5], [0,0,0,1])

	geomTypes = [pb.GEOM_BOX, pb.GEOM_SPHERE, pb.GEOM_CAPSULE, pb.GEOM_CYLINDER]
	
	connectorId  = pb.createRigidBody(pb.GEOM_SPHERE, radius=0.5, position=[0,0,1])
	rbId  = pb.createRigidBody(pb.GEOM_CYLINDER, radius=0.5, height=1, position=[0,0,2])

	# cId = pb.createConstraint(robotId, link_index_map['gripper_link'], 
	# 						  connectorId, -1, 
	# 						  pb.JOINT_FIXED, [0,0,0], 
	# 						  [0,0,0], [1,0,1], [0,0,0,1], [0,0,0,1])

	cId = pb.createConstraint(connectorId, -1, 
							  rbId, -1, 
							  pb.JOINT_FIXED, [0,0,0], 
							  [0,0,0.5], [0,0,-0.5])

	for x in range(10):
		pb.appendAnchor(sbId, rbId, x)

	objIds = []

	#for x in range(10):
	# 	objIds.append(pb.createRigidBody(geomTypes[x % len(geomTypes)], radius=0.5, height=0.5, position=[4,0, 3 + 1.5 *x], halfExtents=[0.5, 0.5, 0.5]))
	
	#objId = pb.createMultiBody(5, boxCId, -1, [0,0,5], [0,0,0,1])
	#pb.appendAnchor(sbId, objId, 0)

	while True:
		if raw_input('TYPE Q TO END\n') in ['q', 'Q']:
			break

		#pb.stepSimulation()
		pb.setRealTimeSimulation(True)


	pb.disconnect()