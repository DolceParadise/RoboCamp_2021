import pybullet as p
import time
import pybullet_data
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
planeId = p.loadURDF("plane.urdf")
p.setGravity(0, 0, -10)
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
#boxId1 = p.loadURDF("sample.urdf",[2,2,1], cubeStartOrientation)

def fib(i):
    if(i==1):
        return 0
    elif(i==2):
        return 1
    else:
        return fib(i-1)+fib(i-2)
i=1
while(True):

    p.stepSimulation()
    if (i==1):
        i = i +1
        continue
    for j in range(fib(i)):
        boxId = p.loadURDF("sphere.urdf",[j,0,5], cubeStartOrientation)
        print(j)
    cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)

    while(cubePos[2] > 0.2):
        p.stepSimulation()
        cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
        continue
    i=i+1