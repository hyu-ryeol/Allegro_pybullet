import pybullet as p
import numpy as np
import time
import pybullet_data
import modern_robotics as mr
import matplotlib.pyplot as plt

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeid = p.loadURDF("plane.urdf", [0, 0, 0])
robotid = p.loadURDF("robot1.urdf",[0,0,0.095],[0,0,0,1])
# objId = p.loadURDF("teddy_vhacd.urdf",[0,0,0],[0,0,0,1])
jointNumber = 21
maxForce = 10
p.setGravity(0,0,-9.8)
finger1_jointIndex = [1,2,3,4]
finger2_jointIndex = [6,7,8,9]
finger3_jointIndex = [11,12,13,14]
finger4_jointIndex = [16,17,18,19]

M_01= np.array([[1,	0,	0,	0],    [0,	0.996194698096079,	0.0871557426981309,	0.0435],  [0,	-0.0871557426981309,	0.996194698096079,	-0.001542],  [0,	0,	0,	1]])
M_02= np.array([[1,	0,	0,	0],	   [0,	1,	0,	0],	   [0,	0,	1,	0.0007],	   [0,	0,	0,	1]])
M_03= np.array([[1,	0,	0,	0],	   [0,	0.996194698096079,	-0.0871557426981309,	-0.0435],	   [0,	0.0871557426981309,	0.996194698096079,	-0.001542],	   [0,	0,	0,	1]])

M_12= np.array([[1,	0,	0,	0],		[0,	1,	0,	0],		[0,	0,	1,	0.0164],		[0,	0,	0,	1]])
M_23= np.array([[1,	0,	0,	0],		[0,	1,	0,	0],		[0,	0,	1,	0.054],		[0,	0,	0,	1]])
M_34= np.array([[1,	0,	0,	0],		[0,	1,	0,	0],		[0,	0,	1,	0.0384],		[0,	0,	0,	1]])
M_45= np.array([[1,	0,	0,	0],		[0,	1,	0,	0],		[0,	0,	1,	0.0267],		[0,	0,	0,	1]])

M4_01=np.array([[-7.79953824603932e-11,	1,	-8.91491310595254e-10,	-0.0182000000000000],	   [0.0871557418066397,	8.94896662499077e-10,	0.996194698174074,	0.0193330000000000],	   [0.996194698174074,	0,	-0.0871557418066397,	-0.0459870000000000],	   [0,	0,	0,	1]])
M4_12=np.array([[1,	0,	0,	-0.027],	   [0,	1,	0,	0.005],	   [0,	0,	1,	0.0399],	   [0,	0,	0,	1]])
M4_23=np.array([[1,	0,	0,	0],	   [0,	1,	0,	0],	   [0,	0,	1,	0.0177],	   [0,	0,	0,	1]])
M4_34=np.array([[1,	0,	0,	0],	   [0,	1,	0,	0],	   [0,	0,	1,	0.0514],	   [0,	0,	0,	1]])
M4_45=np.array([[1,	0,	0,	0],	   [0,	1,	0,	0],	   [0,	0,	1,	0.0423],	   [0,	0,	0,	1]])

M1=np.matmul(np.matmul(np.matmul(np.matmul(M_01,M_12),M_23),M_34),M_45)
M2=np.matmul(np.matmul(np.matmul(np.matmul(M_02,M_12),M_23),M_34),M_45)
M3=np.matmul(np.matmul(np.matmul(np.matmul(M_03,M_12),M_23),M_34),M_45)
M4=np.matmul(np.matmul(np.matmul(np.matmul(M4_01,M4_12),M4_23),M4_34),M4_45)

S1=np.array([[0, 0.0871557426981309, 0.996194698096079, 0.0434688622740562, 0, 0],	[0,0.996194698096079,-0.0871557426981309,-0.0186551432283326,0,   0],	[0,0.996194698096079,-0.0871557426981309,-0.0726551549383900,0,0],		[0,0.996194698096079,-0.0871557426981309,-0.111055152273398,0,0]]).T
S2=np.array([[0,0,1,0,0,0],		[0,1,0,-0.0171000007539988,0,0],		[0,1,0,-0.0711000040173531,0,0],		[0,1,0,-0.109500005841255,0,0]]).T
S3=np.array([[0,-0.0871557426981309,0.996194698096079,-0.0434688622740562,0,0],		[0,0.996194698096079,0.0871557426981309,-0.0186551432283326,0,0],		[0,0.996194698096079,0.0871557426981309,-0.0726551475161611,0,0],		[0,0.996194698096079,0.0871557426981309,-0.111055152273398,0,0]]).T
S4=np.array([[7.79953824603932e-11,-0.0871557418066397,-0.996194698174074,-0.0232674624619271,-0.0181307442555467,0.00158623456456879],			[-8.91491310595254e-10,0.996194698174074,-0.0871557418066397,0.0711270282984123,-0.00115045563636781,-0.0131497689662933],			[1,8.94896662499077e-10,0,6.97164170963295e-11,-0.0779044330120087,-0.0743606090663781],			[1,8.94896662499077e-10,0,7.37253779793818e-11,-0.0823842361569405,-0.125565022241961]]).T

G0=np.array([[1e-4,0.0,0.0], 								[0.0,1e-4,0.0], 							[0.0, 0.0, 0.0001]])
G_1=np.array([[1.01666658333e-06,0.0,0.0],   				[0.0,6.47677333333e-07,0.0],   				[0.0,0.0,1.01666658333e-06]])
G_2=np.array([[7.95654166667e-05,1.7199e-05,8.75875e-06],   [1.7199e-05,2.47088833333e-05,2.413125e-05],[8.75875e-06,2.413125e-05,7.95654166667e-05]])
G_3=np.array([[2.63979183333e-05,6.67968e-06,4.783625e-06], [6.67968e-06,1.34948516667e-05,9.372e-06],  [4.783625e-06,9.372e-06,2.63979183333e-05]])
G_4=np.array([[4.701248e-06,1.255968e-06,1.2936e-06],  	    [1.255968e-06,3.649312e-06,1.7622e-06],   	[1.2936e-06,1.7622e-06,4.701248e-06]])

G4_1=np.array([[1.89273333333e-5,7.16716e-06,5.35568e-06],
			   [7.16716e-06,1.43008213333e-05,6.8068e-06],
			   [5.35568e-06,6.8068e-06,1.89273333333e-05]])
G4_2=np.array([[4.24250866667e-06,1.032087e-06,1.603525e-06],
			   [1.032087e-06,4.52362633333e-06,1.44808125e-06],
			   [1.603525e-06,1.44808125e-06,4.24250866667e-06]])
G4_3=np.array([[4.30439933333e-05,9.57068e-06,5.1205e-06],
			   [9.57068e-06,1.44451933333e-05,1.342825e-05],
			   [5.1205e-06,1.342825e-05,4.30439933333e-05]])
G4_4=np.array([[3.29223173333e-05,8.042076e-06,5.2283e-06],
			   [8.042076e-06,1.47493026667e-5,1.1283525e-5],
			   [5.2283e-06,1.1283525e-5,3.29223173333e-05]])
G1=[G_1,G_2,G_3,G_4]
G2=[G_1,G_2,G_3,G_4]
G3=[G_1,G_2,G_3,G_4]
G4=[G4_1,G4_2,G4_3,G4_4]

# Mass1=mr.MassMatrix(theta_list1,M1,G1,S1)
# print(G1_1)

theta_list1=[-0.02785654223134834, 0.009314805919159919, 0.918914391495951, 0.7941311423884578]
theta_list2=[0.0066785400981930945, -0.005536158235084111, 0.8715494821394858, 0.764692840660485]
theta_list3=[0.06300675323675811, 0.029174675142167622, 0.868913216319781, 0.8087184799534589] 
theta_list4=[0.7299820072825081, 0.4579193740155175, 0.23919718596737496, 0.7686472393976554] 

theta_list1 =[-0.02785654223134834, 0.3, 0.918914391495951, 0.9];
theta_list2 =[0.0066785400981930945,0.3, 0.8715494821394858, 0.9];
theta_list3 =[0,0.4,0.41,0.41];
theta_list4 =[ 1.0, 0.5, 1, 0.7];

T1=mr.FKinSpace(M1,S1,theta_list1)
T2=mr.FKinSpace(M2,S2,theta_list2)
T3=mr.FKinSpace(M3,S3,theta_list3)
T4=mr.FKinSpace(M4,S4,theta_list4)

Xstart=np.array([[ 1,          0,          0,          0         ], 
				[ 0,           1,          0,          0.0553096 ], 
				[ 0,           0,          1,          0.13344238],
				[ 0,           0,          0,          1        ]])

Xend1=np.array(T1)
Xend2=np.array(T2)
Xend3=np.array(T3)
Xend4=np.array(T4)
traj_stepsize=300
traj_time=10

# traj1=mr.JointTrajectory([0,0,0,0],theta_list1,traj_time,traj_stepsize,5)
# traj2=mr.JointTrajectory([0,0,0,0],theta_list2,traj_time,traj_stepsize,5)
# traj3=mr.JointTrajectory([0,0,0,0],theta_list3,traj_time,traj_stepsize,5)
# traj4=mr.JointTrajectory([0,0,0,0],theta_list4,traj_time,traj_stepsize,5)

traj1=mr.ScrewTrajectory(Xstart,Xend1,3,traj_stepsize,3)
# traj2=mr.ScrewTrajectory(Xstart,Xend2,3,traj_stepsize,3)
# traj3=mr.ScrewTrajectory(Xstart,Xend3,3,traj_stepsize,3)
# traj4=mr.ScrewTrajectory(Xstart,Xend4,3,traj_stepsize,3)

# T_sol1=np.array(traj1[0])
# T_sol2=np.array(traj2[0])
# T_sol3=np.array(traj3[0])
# T_sol4=np.array(traj4[0])
i=0
# print(np.array(traj1[1]))
# Disable the motors for torque control:
p.setJointMotorControlArray(robotid,
                                 [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19],
                                 p.VELOCITY_CONTROL,
                                 forces=[10]*20)

while(1):
	
	# q = []
	# dq=[]
	# for i in range(7):
	# 	jointState = p.getJointState(robotid,i)
	# 	q.append(jointState[0])
	# 	# dq.append(jointState[1])
	# print("encoder:",q)
	# # print("velocity:",dq)
	i=i+1	
	T_sol1=np.array(traj1[i])
	# T_sol2=np.array(traj2[i])
	# T_sol3=np.array(traj3[i])
	# T_sol4=np.array(traj4[i])
	# for i in range(0,traj_stepsize):
	# 	traj_1=np.array(traj1[i])
	# 	print(traj_1)
	# 	traj_2=np.array(traj2[i])
	# 	traj_3=np.array(traj3[i])
	# 	traj_4=np.array(traj4[i])

	# torque=[0,0,0,0]
	


   



	# p.setJointMotorControlArray(robotid,
 #                                   finger1_jointIndex,
 #                                   p.TORQUE_CONTROL,
 #                                   forces=[torque[0], torque[1],torque[2],torque[3]])


	print(T_sol1)
	Ik1=np.array(mr.IKinSpace(S1,M1,T_sol1,[0,0,0,0],0.01,0.001))[0]
	# Ik2=np.array(mr.IKinSpace(S2,M2,T_sol2,[0,0,0,0],0.01,0.001))[0]
	# Ik3=np.array(mr.IKinSpace(S3,M3,T_sol3,[0,0,0,0],0.01,0.001))[0]
	# Ik4=np.array(mr.IKinSpace(S4,M4,T_sol4,[0,0,0,0],0.01,0.001))[0]

	# print(Ik1)
	# plt.plot(Ik1[0])
		



	

	p.setJointMotorControlArray(robotid, jointIndices=finger1_jointIndex, controlMode=p.POSITION_CONTROL, targetPositions=Ik1 ,forces=[10]*4)
	# p.setJointMotorControlArray(robotid, jointIndices=finger2_jointIndex, controlMode=p.POSITION_CONTROL, targetPositions=traj_2,forces=[0]*4)
	# p.setJointMotorControlArray(robotid, jointIndices=finger3_jointIndex, controlMode=p.POSITION_CONTROL, targetPositions=traj_3,forces=[0]*4)
	# p.setJointMotorControlArray(robotid, jointIndices=finger4_jointIndex, controlMode=p.POSITION_CONTROL, targetPositions=traj_4,forces=[0]*4)


	# finger1_jointState=np.array(p.getJointStates(robotid,finger1_jointIndex))
	# finger2_jointState=np.array(p.getJointStates(robotid,finger2_jointIndex))
	# finger3_jointState=np.array(p.getJointStates(robotid,finger3_jointIndex))
	# finger4_jointState=np.array(p.getJointStates(robotid,finger4_jointIndex))

	# print(finger1_jointState)
	# print(finger1_jointState[0:4,0])
	# print(finger2_jointState[0:4,1])

	# print(p.getJointInfo(robotid,5))


	


	# finger1_linkState=np.array(p.getLinkState(bodyUniqueId=robotid,linkIndex=5))
	# print(np.round(finger1_linkState[0],3),np.round(finger1_linkState[1],3))
	


	# p.setJointMotorControl2(bodyUniqueId=robotid, 
	# jointIndex=1, 
	# controlMode=p.POSITION_CONTROL,
	# targetPosition = 0,
	# force = 9)


	# p.calculateInverseDynamics(bodyUniqueId=robotid,objPositions=[1,1,1,1],objVelocities=[1,1,1,1], objAccelerations=[0,0,0,0])
 
	# p.calculateInverseKinematics2(bodyUniqueId=robotid,endEffectorLinkIndices=[5,10,15,20],targetPositions=[1,1,1])
	# p.calculateJacobian(bodyUniqueId=robotid,linkindex=5,)



	p.stepSimulation()
	time.sleep(0.01)

p.disconnect(p.GUI)
