import pybullet as p
import numpy as np
import time
import pybullet_data
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeid = p.loadURDF("plane.urdf", [0, 0, 0])
robotid = p.loadURDF("robot1.urdf",[0,0,0.095],[0,0,0,1])
objId = p.loadURDF("teddy_vhacd.urdf",[0,0,0],[0,0,0,1])
jointNumber = 21
maxForce = 10
p.setGravity(0,0,-10)
finger1_jointIndex = [1,2,3,4]
finger2_jointIndex = [6,7,8,9]
finger3_jointIndex = [11,12,13,14]
finger4_jointIndex = [16,17,18,19]


while(1):
	# q = []
	# dq=[]
	# for i in range(7):
	# 	jointState = p.getJointState(robotid,i)
	# 	q.append(jointState[0])
	# 	# dq.append(jointState[1])
	# print("encoder:",q)
	# # print("velocity:",dq)


	finger1_jointState=np.array(p.getJointStates(robotid,finger1_jointIndex))
	finger2_jointState=np.array(p.getJointStates(robotid,finger2_jointIndex))
	finger3_jointState=np.array(p.getJointStates(robotid,finger3_jointIndex))
	finger4_jointState=np.array(p.getJointStates(robotid,finger4_jointIndex))

	# print(finger1_jointState)
	# print(finger1_jointState[0:4,0])
	# print(finger2_jointState[0:4,1])

	# print(p.getJointInfo(robotid,5))


	p.setJointMotorControlArray(robotid, jointIndices=finger1_jointIndex, controlMode=p.POSITION_CONTROL, targetPositions=[-0.02785654223134834, 0.009314805919159919, 0.918914391495951, 0.7941311423884578] ,forces=[10]*4)
	p.setJointMotorControlArray(robotid, jointIndices=finger2_jointIndex, controlMode=p.POSITION_CONTROL, targetPositions=[0.0066785400981930945, -0.005536158235084111, 0.8715494821394858, 0.764692840660485] ,forces=[10]*4)
	p.setJointMotorControlArray(robotid, jointIndices=finger3_jointIndex, controlMode=p.POSITION_CONTROL, targetPositions=[0.06300675323675811, 0.029174675142167622, 0.868913216319781, 0.8087184799534589] ,forces=[10]*4)
	p.setJointMotorControlArray(robotid, jointIndices=finger4_jointIndex, controlMode=p.POSITION_CONTROL, targetPositions=[0.7299820072825081, 0.4579193740155175, 0.23919718596737496, 0.7686472393976554] ,forces=[20]*4)

	finger1_linkState=np.array(p.getLinkState(bodyUniqueId=robotid,linkIndex=5))
	print(np.round(finger1_linkState[0],3),np.round(finger1_linkState[1],3))
	


	# p.setJointMotorControl2(bodyUniqueId=robotid, 
	# jointIndex=1, 
	# controlMode=p.POSITION_CONTROL,
	# targetPosition = 0,
	# force = 9)


	# p.calculateInverseDynamics(bodyUniqueId=robotid,objPositions=[0.1,0.03,0.1],objVelocities=1,objAccelerations=0)
 
	# p.calculateInverseKinematics2(bodyUniqueId=robotid,endEffectorLinkIndices=[5,10,15,20],targetPositions=[1,1,1])
	# p.calculateJacobian(bodyUniqueId=robotid,linkindex=5,)



	p.stepSimulation()
	time.sleep(0.01)
p.disconnect(p.GUI)
