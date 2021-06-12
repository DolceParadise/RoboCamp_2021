
import pybullet as p
import pybullet_data
import os
file_path = os.getcwd()
file_name = "Week1_BonusTask1_Code1.urdf"
physicsClient = p.connect(p.GUI)
p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), 0, 0, 0)
robot = p.loadURDF(os.path.join(file_path, file_name))
p.resetBasePositionAndOrientation(robot, [0, 0, 1], [0, 0, 0, 0.707])
p.setGravity(0,0,0)

while(True):
	p.stepSimulation()
