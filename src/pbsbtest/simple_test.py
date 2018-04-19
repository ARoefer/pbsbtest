#!/usr/bin/env python
import rospy
import pybullet as pb
from pbsbtest.utils import res_pkg_path

if __name__ == '__main__':
	physicsClient = pb.connect(pb.GUI, options="--opengl2")#or p.DIRECT for non-graphical version
	pb.setGravity(0,0, -9.81)
	planeId =  pb.loadURDF(res_pkg_path("package://pbsbtest/urdf/plane.urdf"))

	sbId = pb.loadSoftBody(res_pkg_path('package://pbsbtest/meshes/weird_sponge.obj'), 0.5, 1, 0.04)
	#pb.resetBasePositionAndOrientation(sbId, [0,0,2], [0,0,0,1])

	boxCId = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=[0.3]*3)
	
	for x in range(10):
		objBId = pb.createMultiBody(5, boxCId, -1, [0,0,5 + x*1.5], [0,0,0,1])

	pb.setRealTimeSimulation(True)

	lol = raw_input('PRESS ENTER TO END')

	pb.disconnect()