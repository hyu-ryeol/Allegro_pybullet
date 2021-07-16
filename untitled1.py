import pybullet as p
import numpy as np
import time
import pybullet_data
import modern_robotics as mr

p.connect(p.GUI)

finger1_jointIndex = [1,2,3,4]
finger2_jointIndex = [6,7,8,9]
finger3_jointIndex = [11,12,13,14]
finger4_jointIndex = [16,17,18,19]

p.setAdditionalSearchPath(pybullet_data.getDataPath())
planeid = p.loadURDF("plane.urdf", [0, 0, 0])
robotid = p.loadURDF("robot1.urdf",[0,0,0.095],[0,0,0,1])

# Disable the motors for torque control:
p.setJointMotorControlArray(robotid,jointIndices=[1,2,3,4,6,7,8,9,11,12,13,14,16,17,18,19],
                           controlMode=p.VELOCITY_CONTROL,
                           forces=[0]*16)

# timeStepId = p.addUserDebugParameter("timeStep", 0.001, 0.1, 0.01)
# jointId_11 = p.addUserDebugParameter("joint11", -2, 1.5,2)
# jointId_12 = p.addUserDebugParameter("joint12", 0.001, 1.5, 0)
# jointId_13 = p.addUserDebugParameter("joint13", 0.001, 1.5, 0)
# jointId_14 = p.addUserDebugParameter("joint14", 0.001, 1.5, 0)
# jointId_21 = p.addUserDebugParameter("joint21", 0.001, 1.5, 0)
# jointId_22 = p.addUserDebugParameter("joint22", 0.001, 1.5, 0)
# jointId_23 = p.addUserDebugParameter("joint23", 0.001, 1.5, 0)
# jointId_24 = p.addUserDebugParameter("joint24", 0.001, 1.5, 0)
# jointId_31 = p.addUserDebugParameter("joint31", 0.001, 1.5, 0)
# jointId_32 = p.addUserDebugParameter("joint32", 0.001, 1.5, 0)
# jointId_33 = p.addUserDebugParameter("joint33", 0.001, 1.5, 0)
# jointId_34 = p.addUserDebugParameter("joint34", 0.001, 1.5, 0)
# jointId_41 = p.addUserDebugParameter("joint41", 0.001, 1.5, 0)
# jointId_42 = p.addUserDebugParameter("joint42", 0.001, 1.5, 0)
# jointId_43 = p.addUserDebugParameter("joint43", 0.001, 1.5, 0)
# jointId_44 = p.addUserDebugParameter("joint44", 0.001, 1.5, 0.01)

useRealTimeSim = False
p.setRealTimeSimulation(useRealTimeSim)
p.setGravity(0,0,0)
g=np.array([0,0,0])


# M Lists for each finger (finger1,2,3 has same M list)
M_01= np.array([[1,	0,	0,	0],    
				[0,	0.996194698096079,	0.0871557426981309,	0.04426],   
				[0,	-0.0871557426981309,	0.996194698096079,	0.1047],  
				[0,	0,	0,	1]])
M_02= np.array([[1,	0,	0,	0],	   
				[0,	1,	0,	0],	   
				[0,	0,	1,	0.10701],	   	
				[0,	0,	0,	1]])
M_03= np.array([[1,	0,	0,	0],	   
				[0,	0.996194698096079,	-0.0871557426981309,	-0.0435],	   
				[0,	0.0871557426981309,	0.996194698096079,	0.10475],	   
				[0,	0,	0,	1]])

M_12= np.array([[1,	0,	0,	0],		
				[0,	1,	0,	0],		
				[0,	0,	1,	0.0375],		
				[0,	0,	0,	1]])
M_23= np.array([[1,	0,	0,	0],		
				[0,	1,	0,	0],		
				[0,	0,	1,	0.0533],		
				[0,	0,	0,	1]])
M_34= np.array([[1,	0,	0,	0],		
				[0,	1,	0,	0],			
				[0,	0,	1,	0.0217],		
				[0,	0,	0,	1]])
M_45= np.array([[1,	0,	0,	0],		
				[0,	1,	0,	0],		
				[0,	0,	1,	0.0169],		
				[0,	0,	0,	1]])

M4_01=np.array([[-7.79953824603932e-11,	1,	-8.91491310595254e-10,	-0.016771],	   
				[0.0871557418066397,	8.94896662499077e-10,	0.996194698174074,	0.021121],	   
				[0.996194698174074,	0,	-0.0871557418066397,	0.020735],	   
				[0,	0,	0,	1]])
M4_12=np.array([[1,	0,	0,	-0.0098],	   
				[0,	1,	0,	-0.0029],	   
				[0,	0,	1,	0.0335],	  
				 [0,	0,	0,	1]])
M4_23=np.array([[1,	0,	0,	0],	   [0,	1,	0,	0],	   [0,	0,	1,	0.0261],	   [0,	0,	0,	1]])
M4_34=np.array([[1,	0,	0,	0],	   [0,	1,	0,	0],	   [0,	0,	1,	0.0496],	   [0,	0,	0,	1]])
M4_45=np.array([[1,	0,	0,	0],	   [0,	1,	0,	0],	   [0,	0,	1,	0.0297],	   [0,	0,	0,	1]])

M1=np.array([M_01,M_12,M_23,M_34,M_45])
M2=np.array([M_02,M_12,M_23,M_34,M_45])
M3=np.array([M_03,M_12,M_23,M_34,M_45])
M4=np.array([M4_01,M4_12,M4_23,M4_34,M4_45])

#S lists for each finger
S1=np.array([[0, 0.0871557426981309, 0.996194698096079, 0.0434688622740562, 0, 0],	[0,0.996194698096079,-0.0871557426981309,-0.0186551432283326,0,   0],	[0,0.996194698096079,-0.0871557426981309,-0.0726551549383900,0,0],		[0,0.996194698096079,-0.0871557426981309,-0.111055152273398,0,0]]).T
S2=np.array([[0,0,1,0,0,0],		[0,1,0,-0.0171000007539988,0,0],		[0,1,0,-0.0711000040173531,0,0],		[0,1,0,-0.109500005841255,0,0]]).T
S3=np.array([[0,-0.0871557426981309,0.996194698096079,-0.0434688622740562,0,0],		[0,0.996194698096079,0.0871557426981309,-0.0186551432283326,0,0],		[0,0.996194698096079,0.0871557426981309,-0.0726551475161611,0,0],		[0,0.996194698096079,0.0871557426981309,-0.111055152273398,0,0]]).T
S4=np.array([[7.79953824603932e-11,-0.0871557418066397,-0.996194698174074,-0.0232674624619271,-0.0181307442555467,0.00158623456456879],			[-8.91491310595254e-10,0.996194698174074,-0.0871557418066397,0.0711270282984123,-0.00115045563636781,-0.0131497689662933],			[1,8.94896662499077e-10,0,6.97164170963295e-11,-0.0779044330120087,-0.0743606090663781],			[1,8.94896662499077e-10,0,7.37253779793818e-11,-0.0823842361569405,-0.125565022241961]]).T
# print(np.shape(Slist))

#B lists for each finger
# print(mr.Adjoint(M_12))
# print(S1[:,0])
M_1=np.matmul(np.matmul(np.matmul(np.matmul(M_01,M_12),M_23),M_34),M_45)
M_2=np.matmul(np.matmul(np.matmul(np.matmul(M_02,M_12),M_23),M_34),M_45)
M_3=np.matmul(np.matmul(np.matmul(np.matmul(M_03,M_12),M_23),M_34),M_45)
M_4=np.matmul(np.matmul(np.matmul(np.matmul(M4_01,M4_12),M4_23),M4_34),M4_45)


B11=np.dot(mr.Adjoint(np.linalg.inv(M_01)),S1[:,0])
B12=np.dot(mr.Adjoint(np.linalg.inv(M_12)),S1[:,1])
B13=np.dot(mr.Adjoint(np.linalg.inv(M_23)),S1[:,2])
B14=np.dot(mr.Adjoint(np.linalg.inv(M_34)),S1[:,3])
B1=np.array([B11,B12,B13,B14]).T

B21=np.dot(mr.Adjoint(np.linalg.inv(M_02)),S2[:,0])
B22=np.dot(mr.Adjoint(np.linalg.inv(M_12)),S2[:,1])
B23=np.dot(mr.Adjoint(np.linalg.inv(M_23)),S2[:,2])
B24=np.dot(mr.Adjoint(np.linalg.inv(M_34)),S2[:,3])
B2=np.array([B21,B22,B23,B24]).T

B31=np.dot(mr.Adjoint(np.linalg.inv(M_01)),S3[:,0])
B32=np.dot(mr.Adjoint(np.linalg.inv(M_12)),S3[:,1])
B33=np.dot(mr.Adjoint(np.linalg.inv(M_23)),S3[:,2])
B34=np.dot(mr.Adjoint(np.linalg.inv(M_34)),S3[:,3])
B3=np.array([B31,B32,B33,B34]).T

B41=np.dot(mr.Adjoint(M4_01),S4[:,0])
B42=np.dot(mr.Adjoint(M4_12),S4[:,1])
B43=np.dot(mr.Adjoint(M4_23),S4[:,2])
B44=np.dot(mr.Adjoint(M4_34),S4[:,3])
B4=np.array([B41,B42,B43,B44]).T


#G lists for each finger

G0=np.array([[1e-4,0.0,0.0], 								[0.0,1e-4,0.0], 							[0.0, 0.0, 0.0001]])
G_1=np.array([[1.01666658333e-06,0.0,0.0],   				[0.0,6.47677333333e-07,0.0],   				[0.0,0.0,1.01666658333e-06]])
G_2=np.array([[7.95654166667e-05,1.7199e-05,8.75875e-06],   [1.7199e-05,2.47088833333e-05,2.413125e-05],[8.75875e-06,2.413125e-05,7.95654166667e-05]])
G_3=np.array([[2.63979183333e-05,6.67968e-06,4.783625e-06], [6.67968e-06,1.34948516667e-05,9.372e-06],  [4.783625e-06,9.372e-06,2.63979183333e-05]])
G_4=np.array([[4.701248e-06,1.255968e-06,1.2936e-06],  	    [1.255968e-06,3.649312e-06,1.7622e-06],   	[1.2936e-06,1.7622e-06,4.701248e-06]])
G_5=np.array(np.dot(np.eye(3),9.68e-07))
G4_1=np.array([[1.89273333333e-5,7.16716e-06,5.35568e-06],	  [7.16716e-06,1.43008213333e-05,6.8068e-06],	  [5.35568e-06,6.8068e-06,1.89273333333e-05]])
G4_2=np.array([[4.24250866667e-06,1.032087e-06,1.603525e-06],   [1.032087e-06,4.52362633333e-06,1.44808125e-06],	   [1.603525e-06,1.44808125e-06,4.24250866667e-06]])
G4_3=np.array([[4.30439933333e-05,9.57068e-06,5.1205e-06],	   [9.57068e-06,1.44451933333e-05,1.342825e-05],	   [5.1205e-06,1.342825e-05,4.30439933333e-05]])
G4_4=np.array([[3.29223173333e-05,8.042076e-06,5.2283e-06],	   [8.042076e-06,1.47493026667e-5,1.1283525e-5],	   [5.2283e-06,1.1283525e-5,3.29223173333e-05]])
Im_1=np.diag([0.0119,0.0119,0.0119])
Im_2=np.diag([0.065,0.065,0.065])
Im_3=np.diag([0.0355,0.0355,0.0355])
Im_4=np.diag([0.0096,0.0096,0.0096])
Im_5=np.diag([0.0168,0.0168,0.0168])
Im4_1=np.diag([0.0176,0.0176,0.0176])
Im4_2=np.diag([0.0119,0.0119,0.0119])
Im4_3=np.diag([0.038,0.038,0.038])
Im4_4=np.diag([0.0388,0.0388,0.0388])

zero=np.zeros((3,3))
G_1=np.vstack([ np.hstack ([G_1,zero]) , np.hstack([zero,Im_1])])
G_2=np.vstack([ np.hstack ([G_2,zero]) , np.hstack([zero,Im_2])])
G_3=np.vstack([ np.hstack ([G_3,zero]) , np.hstack([zero,Im_3])])
G_4=np.vstack([ np.hstack ([G_4,zero]) , np.hstack([zero,Im_4])])
G_5=np.vstack([ np.hstack ([G_5,zero]) , np.hstack([zero,Im_5])])

G4_1=np.vstack([ np.hstack ([G4_1,zero]) , np.hstack([zero,Im4_1])])
G4_2=np.vstack([ np.hstack ([G4_2,zero]) , np.hstack([zero,Im4_2])])
G4_3=np.vstack([ np.hstack ([G4_3,zero]) , np.hstack([zero,Im4_3])])
G4_4=np.vstack([ np.hstack ([G4_4,zero]) , np.hstack([zero,Im4_4])])

G1=np.array([G_1,G_2,G_3,G_4])
G2=np.array([G_1,G_2,G_3,G_4])
G3=np.array([G_1,G_2,G_3,G_4])
G4=np.array([G4_1,G4_2,G4_3,G4_4])


# print(np.round(M3,7))


des_thetalist1=[-0.02785654223134834, 0.009314805919159919, 0.918914391495951, 0.7941311423884578]
des_thetalist2=[0.0066785400981930945, -0.005536158235084111, 0.8715494821394858, 0.764692840660485]
des_thetalist3=[0.06300675323675811, 0.029174675142167622, 0.868913216319781, 0.8087184799534589] 
des_thetalist4=[0.7299820072825081, 0.4579193740155175, 0.23919718596737496, 0.7686472393976554] 
# G1=np.array(G1,dtype=float)
# print(np.round(G1,6))

# Fi = np.array(Ftip).copy()
# taulist = np.zeros(n)
Slist=np.hstack([S1])
while (1):
	timeStep=0.001
	# timeStep = p.readUserDebugParameter(timeStepId)
	# des_joint11 = p.readUserDebugParameter(jointId_11);
	# des_joint12 = p.readUserDebugParameter(jointId_12);
	# des_joint13 = p.readUserDebugParameter(jointId_13);
	# des_joint14 = p.readUserDebugParameter(jointId_14);
	# des_joint21 = p.readUserDebugParameter(jointId_21);
	# des_joint22 = p.readUserDebugParameter(jointId_22);
	# des_joint23 = p.readUserDebugParameter(jointId_23);
	# des_joint24 = p.readUserDebugParameter(jointId_24);
	# des_joint31 = p.readUserDebugParameter(jointId_31);
	# des_joint32 = p.readUserDebugParameter(jointId_32);
	# des_joint33 = p.readUserDebugParameter(jointId_33);
	# des_joint34 = p.readUserDebugParameter(jointId_34);
	# des_joint41 = p.readUserDebugParameter(jointId_41);
	# des_joint42 = p.readUserDebugParameter(jointId_42);
	# des_joint43 = p.readUserDebugParameter(jointId_43);
	# des_joint44 = p.readUserDebugParameter(jointId_44);
	# des_thetalist1=[des_joint11,des_joint12,des_joint13,des_joint14]
	# des_thetalist2=[des_joint21,des_joint22,des_joint23,des_joint24]
	# des_thetalist3=[des_joint31,des_joint32,des_joint33,des_joint34]
	# des_thetalist4=[des_joint41,des_joint42,des_joint43,des_joint44]


	# p.setTimeStep(timeStep)
	dt=timeStep
	# get joint states from encoder
	finger1_jointState=np.array(p.getJointStates(robotid,finger1_jointIndex))
	finger2_jointState=np.array(p.getJointStates(robotid,finger2_jointIndex))
	finger3_jointState=np.array(p.getJointStates(robotid,finger3_jointIndex))
	finger4_jointState=np.array(p.getJointStates(robotid,finger4_jointIndex))

	# actual joint angles
	thetalist1=finger1_jointState[0:4,0]
	thetalist2=finger2_jointState[0:4,0]
	thetalist3=finger3_jointState[0:4,0]
	thetalist4=finger4_jointState[0:4,0]

	# actual joint velocities
	dthetalist1=finger1_jointState[0:4,1]
	dthetalist2=finger2_jointState[0:4,1]
	dthetalist3=finger3_jointState[0:4,1]
	dthetalist4=finger4_jointState[0:4,1]


	#Wrench(F)
	des_F=[0,0,0,0,0,-10]
	o=[0,0,0,0]
	#Jacobian
	J=np.array(mr.JacobianBody(B1,thetalist1))
	Ja=np.vstack((o,o,o,J[3,:],J[4,:],J[5,:]))
	tau= np.dot(Ja.T, des_F)
	# print(thetalist1)
	# print(Ja.T)
	# print(tau)
	
	# desired  joint velociy&acceleration
	des_dthetalist1=[0,0,0,0]
	des_ddthetalist1=[0,0,0,0]
	Kp=600
	Ki=0
	Kd=10
	Kp4=800
	Ki4=0
	Kd4=10
	
	
	eint=np.array([0,0,0,0])

	e1 = np.subtract(des_thetalist1,thetalist1)
	e2 = np.subtract(des_thetalist2,thetalist2)
	e3 = np.subtract(des_thetalist3,thetalist3)
	e4 = np.subtract(des_thetalist4,thetalist4)

	# Computed Torque Control
	# torque1= np.dot(mr.MassMatrix(thetalist1, M1, G1, S1), Kp * e1 + Ki * (np.array(eint) + e1) + Kd * np.subtract(des_dthetalist1, dthetalist1)) \
	# 		+ mr.InverseDynamics(thetalist1, dthetalist1, des_ddthetalist1, g, \
	# 			[0, 0, 0, 0, 0, 0], M1, G1, S1)




	
	torque1=mr.ComputedTorque(thetalist1,dthetalist1,eint,g,M1,G1,S1,des_thetalist1,des_dthetalist1,des_ddthetalist1,Kp,Ki,Kd)
	torque2=mr.ComputedTorque(thetalist2,dthetalist2,eint,g,M2,G2,S2,des_thetalist2,des_dthetalist1,des_ddthetalist1,Kp,Ki,Kd)
	torque3=mr.ComputedTorque(thetalist3,dthetalist3,eint,g,M3,G3,S3,des_thetalist3,des_dthetalist1,des_ddthetalist1,Kp,Ki,Kd)
	torque4=mr.ComputedTorque(thetalist4,dthetalist4,eint,g,M4,G4,S4,des_thetalist4,des_dthetalist1,des_ddthetalist1,Kp4,Ki4,Kd4)
	#torque=np.array(torque3,dtype=float)

	# print(np.round(torque,3))       
	p.setJointMotorControlArray(robotid,
                           jointIndices=[1,2,3,4,6,7,8,9,11,12,13,14,16,17,18,19],
                           controlMode=p.TORQUE_CONTROL,
                           forces= [torque1[0], torque1[1],torque1[2],torque1[3],
                           		   torque2[0], torque2[1],torque2[2],torque2[3],
                           		   torque3[0], torque3[1],torque3[2],torque3[3],
                           		   torque4[0], torque4[1],torque4[2],torque4[3]])
	# p.setJointMotorControlArray(robotid,
 #                           jointIndices=[1,2,3,4],
 #                           controlMode=p.TORQUE_CONTROL,
 #                           forces= [tau[0],tau[1],tau[2],tau[3]])

	error=np.array(e4,dtype=float)
	print("error=",np.round(error,3))
	# p.resetJointState(robotid,[1,2,3,4,6,7,8,9,11,12,13,14,16,17,18,19])
	p.stepSimulation()
	time.sleep(timeStep)

		
p.disconnect(p.GUI)
