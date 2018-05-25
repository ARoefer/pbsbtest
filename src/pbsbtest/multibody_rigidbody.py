import pybullet as p
import time

p.connect(p.GUI)
p.loadURDF("plane.urdf",[0,0,-0.3])
kukaId = p.loadURDF("kuka_iiwa/model.urdf",[0,0,0])
cube = p.createRigidBody(p.GEOM_BOX, halfExtents=[0.02]*3, position=[1,0,1]) #p.loadURDF(res_pkg_path("package://pbsbtest/urdf/cube_small.urdf"), [-0.5,0,0.5], useMaximalCoordinates=False)

fix_cid = p.createConstraint(kukaId,5,cube,-1,p.JOINT_FIXED,[0,0,0],[0,0.2,0],[0,0,0],[0,0,0,1],[0,0,0,1])
p.changeConstraint(fix_cid,maxForce=1)

while 1:
	p.stepSimulation()
	time.sleep(0.001)