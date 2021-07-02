import matplotlib.pyplot as plt
import numpy as np
import pybullet as p
import time
import pybullet_data
import cv2
p.connect(p.GUI)  #or p.SHARED_MEMORY or p.DIRECT
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")
ramp=p.loadURDF("dabba.urdf")
p.setGravity(0, 0, -10)
carpos = [9, 0, 0.1]
count = 0 
car = p.loadURDF("husky/husky.urdf", carpos[0], carpos[1], carpos[2])
numJoints = p.getNumJoints(car)
print (p.getBasePositionAndOrientation(car))
for joint in range(numJoints):
  print(p.getJointInfo(car, joint))
maxForce = 100 #Newton
#p.applyExternalForce(car,3,[100,0,0],)
while (1):
    rKey = ord('r')
    aKey = ord('a')
    cKey = ord('c')
    keys = p.getKeyboardEvents()
    targetVel = 10 + count   #rad/s
    for k, v in keys.items():
        if (k == p.B3G_UP_ARROW and (v & p.KEY_IS_DOWN)): 
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL, targetVelocity =targetVel,force = maxForce)          
            p.stepSimulation()
        if (k == p.B3G_UP_ARROW and (v & p.KEY_WAS_RELEASED)):
            targetVel = 0
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            p.stepSimulation()
        if (k == p.B3G_DOWN_ARROW and (v & p.KEY_IS_DOWN)):
            targetVel = -targetVel
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)            
            p.stepSimulation()
        if (k == p.B3G_DOWN_ARROW and (v & p.KEY_WAS_RELEASED)):
            targetVel = 0
            for joint in range(2, 6):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,targetVelocity = targetVel,force = maxForce)
            p.stepSimulation()
        if (k == rKey and (v & p.KEY_IS_DOWN)):
            p.setJointMotorControlArray(car,[2,3,4,5],controlMode=p.VELOCITY_CONTROL,targetVelocities = [targetVel,-targetVel,targetVel,-targetVel] ,forces=[maxForce, maxForce, maxForce, maxForce])
            p.stepSimulation()
        if (k == aKey and (v & p.KEY_WAS_RELEASED)):
            count = count + 1 
            p.setJointMotorControlArray(car,[2,3,4,5],controlMode=p.VELOCITY_CONTROL,targetVelocities = [targetVel,-targetVel,targetVel,targetVel] ,forces=[maxForce, maxForce, maxForce, maxForce])
            p.stepSimulation()
        if (k == cKey and (v & p.KEY_WAS_RELEASED)):
            # width = 512
            # height = 512
            # fov = 60
            # aspect = width / height
            # near = 0.02
            # far = 5
            # view_matrix = p.computeViewMatrix([0, 0, 2], [0, 0, 0], [1, 0, 0])
            # projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
            # images = p.getCameraImage(width, height, view_matrix,projection_matrix,shadow=True,renderer=p.ER_BULLET_HARDWARE_OPENGL)
            # rgb_opengl = np.reshape(images[2], (height, width, 4)) * 1. / 255.
            # cv2.imshow('rgb',rgb_opengl)
            # cv2.waitKey(0)
            # p.stepSimulation()
            width = 512
            height = 512
            fov = 60
            aspect = width / height
            near = 0.02
            far = 5
            position = p.getBasePositionAndOrientation(car)
            front_rightposition = p.getLinkState(car, 3)
            front_leftposition = p.getLinkState(car, 5)
            vectorpointx = front_leftposition[0][0] - front_rightposition[0][0]
            vectorpointy = front_leftposition[0][1] - front_rightposition[0][1]
            vectorpointz = front_leftposition[0][2] - front_rightposition[0][2]
            view_matrix = p.computeViewMatrix(position[0], [vectorpointx, vectorpointy, vectorpointz] , [0, 0, 1])
            projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
            images = p.getCameraImage(width,height,view_matrix, projection_matrix,shadow=True,renderer=p.ER_BULLET_HARDWARE_OPENGL)
            rgb_opengl = np.reshape(images[2], (height, width, 4)) * 1. / 255.
            depth_buffer_opengl = np.reshape(images[3], [width, height])
            depth_opengl = far * near / (far - (far - near) * depth_buffer_opengl)
            seg_opengl = np.reshape(images[4], [width, height]) * 1. / 255.
            cv2.imshow('rgb',rgb_opengl)
            cv2.waitKey(0)
            p.stepSimulation()
p.getContactPoints(car)
p.disconnect()