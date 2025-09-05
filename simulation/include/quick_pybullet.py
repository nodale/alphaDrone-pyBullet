import pybullet as p
import time
import pybullet_data
import math

def init_pybullet():
    try:
    physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally

    p.setGravity(0,0,-9.81)
    planeId = p.loadURDF("plane.urdf")
    startPos = [0,0,1]
    startOrientation = p.getQuaternionFromEuler([math.pi/2,0,0])

    boxId = p.loadURDF("urdf/preAlphaDrone.urdf",startPos, startOrientation)

    p.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
    
    except:
        print("error in init_pybullet; path is include/quick_pybullet")
