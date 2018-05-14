#!/usr/bin/env python
import rospy
import pybullet as pb
from pbsbtest.utils import res_pkg_path


if __name__ == '__main__':
	physicsClient = pb.connect(pb.GUI, options="--opengl2")#or p.DIRECT for non-graphical version
	pb.setGravity(0,0, -9.81)
	planeId =  pb.loadURDF(res_pkg_path("package://pbsbtest/urdf/plane.urdf"))
	#robotId = pb.loadURDF(res_pkg_path("package://fetch_description/robots/fetch.urdf"), useFixedBase=1)

	sbId = pb.loadSoftBody(res_pkg_path('package://pbsbtest/meshes/weird_sponge.obj'), 0.5, 1, 0.04)
	pb.resetBasePositionAndOrientation(sbId, [0,0,2], [0,0,0,1])

	geomTypes = [pb.GEOM_BOX, pb.GEOM_SPHERE, pb.GEOM_CAPSULE, pb.GEOM_CYLINDER]
	
	rbId = pb.createRigidBody(pb.GEOM_SPHERE, radius=0.5, height=0.5, position=[0,0,0.5])

	objIds = []

	for x in range(10):
	 	objIds.append(pb.createRigidBody(geomTypes[x % len(geomTypes)], radius=0.5, height=0.5, position=[4,0, 3 + 1.5 *x], halfExtents=[0.5, 0.5, 0.5]))
	
	#objId = pb.createMultiBody(5, boxCId, -1, [0,0,5], [0,0,0,1])
	#pb.appendAnchor(sbId, objId, 0)

	pb.setRealTimeSimulation(True)

	lol = raw_input('PRESS ENTER TO END')

	pb.disconnect()