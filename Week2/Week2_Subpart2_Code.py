import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)  
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
#these are the pre required conditions for the task.
ramp=p.loadURDF("wedge.urdf")
p.setGravity(0, 0, -10)
p.changeDynamics(ramp,-1,lateralFriction=0.5)

huskypos = [2, 0, 0.1]
husky = p.loadURDF("husky/husky.urdf", huskypos[0], huskypos[1], huskypos[2])
counter = 0
targetVel = 10 
'''
1.print Number of Joints and joint info and state

2.Get the user input about the control function they 
want to simulate and see.(its upto you, it can be a string / int anything but just leave
a comment with the legend to your menu)

'''
nb_joints = p.getNumJoints(husky)
print("The robot is made of {} joints.".format(nb_joints))

for i in range (0 , nb_joints):
	joint_index = i
	joint_info = p.getJointInfo(husky, joint_index)
	print("Joint index: {}".format(joint_info[0]))
	print("Joint name: {}".format(joint_info[1]))
	print("Joint type: {}".format(joint_info[2]))
	print("First position index: {}".format(joint_info[3]))
	print("First velocity index: {}".format(joint_info[4]))
	print("flags: {}".format(joint_info[5]))
	print("Joint damping value: {}".format(joint_info[6]))
	print("Joint friction value: {}".format(joint_info[7]))
	print("Joint positional lower limit: {}".format(joint_info[8]))
	print("Joint positional upper limit: {}".format(joint_info[9]))
	print("Joint max force: {}".format(joint_info[10]))
	print("Joint max velocity {}".format(joint_info[11]))
	print("Name of link: {}".format(joint_info[12]))
	print("Joint axis in local frame: {}".format(joint_info[13]))
	print("Joint position in parent frame: {}".format(joint_info[14]))
	print("Joint orientation in parent frame: {}".format(joint_info[15]))
	print("Parent link index: {}".format(joint_info[16]))

print("\n 1) Torque Control \n 2) Force Control ")
choice = int(input("Enter 1 or 2 based on your choice"))

#function to be filled to implement torque control
def Torque_control():

	# find this value to climb the ramp without sliping and topling
	optimal_torque_value = -250
	'''
	this function should have the setJointMotorControl in TORQUE_CONTROL configuration
    with forc = optimal_force_value
    ''' 
	for joint in range(2, 6):
                p.setJointMotorControl2(husky, joint, p.TORQUE_CONTROL,targetVelocity = -targetVel,force = optimal_torque_value)


#function to be filled to implement velocity control
def Velocity_control():
	# Keep a constant non zero value for maxForce and try getting the velocity that makes it climb the ramp.
	maxForce = 1000 

	# find this value to climb the ramp without sliping
	optimal_velocity_value = 0 
	'''
	this function should have the setJointMotorControl in VELOCITY_CONTROL configuration
	with targetvelocity = optimal_velocity_value 
	'''
	for joint in range(2, 6):
                p.setJointMotorControl2(husky, joint, p.VELOCITY_CONTROL,targetVelocity = -targetVel,force = maxForce)


while (1):
	time.sleep(.01)
	'''
	1.Here call either the Torque_control function or Velocity_control 
	  function according to the initial user choice and apply the optimal velocity/Torque
	  to the motors that you have found by experimentation.

	2.print base state and velocity 100 iteration steps once.
	'''
	if (choice == 1):
		Torque_control()
	else: 
		Velocity_control()

	counter = counter + 1
	if (counter % 100 == 0):
		print("The base position and orientation is {} and its velocity is {}".format(p.getBasePositionAndOrientation(husky),p.getBaseVelocity(husky)))
	p.stepSimulation()





p.disconnect()