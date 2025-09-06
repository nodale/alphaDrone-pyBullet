from dataclasses import dataclass

import pybullet as p
import numpy as np

import time
import pybullet_data
import math

@dataclass QuickBullet(QuickMav):

    def __init__(self, address='localhost:14550', baudrate=57600, modelPath='urdf/preAlphaDrone.urdf', worldPath='plane.urdf'):
        super().__init__(address=address, baudrate=baudrate)
        self.sendHeartbeat()

        self.accField = np.array([[0, 0, -9.81]])

        try:
        _physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally

        p.setGravity(accField[0], accField[1], accField[2])
        _planeId = self.p.loadURDF(worldPath)
        _startPos = [0,0,1]
        _startOrientation = self.p.getQuaternionFromEuler([math.pi/2,0,0])

        self.object = p.loadURDF(modelPath, _startPos, _startOrientation)

        p.resetBasePositionAndOrientation(self.object, _startPos, _startOrientation)
        
        self.initSimState()

        except:
            print("error in init_pybullet; path is include/quick_pybullet")

    def initSimState(self):
        self.simPos, self.simQ = p.getBasePositionAndOrientation(self.object)
        self.simVel, self.simAngVel = p.getBaseVelocity(self.object)

        self.simPosP, self.simQP = p.getBasePositionAndOrientation(self.object)
        self.simVelP, self.simAngVelP = p.getBaseVelocity(self.object)

        self.timeC = time.time()
        self.timeP = time.time()
        self.dt = 0.1

    def getSimState(self):
        self.simPosP, self.simQP = self.simPos, self.simQ
        self.simVelP, self.simAngVelP = self.simVel, self.simAngVel

        self.simPos, self.simQ = p.getBasePositionAndOrientation(self.object)
        self.simVel, self.simAngVel = p.getBaseVelocity(self.object)

        self.timeC = time.time()
        self.dt = self.timeC - self.timeP
        self.timeP = self.timeC

    def addNoise(self, obj, center=0.0, amplitude=0.008, dim=3)
        obj += np.random.normal(center, amplitude, dim) 

    def getAccelerometer(self):
        _R = np.array(p.getMatrixFromQuaternion(self.simQ)).reshape(3,3)
        _accWorld = (np.array(self.simVel) - np.array(self.simVelP)) / self.dt
        self.simAcc = _R.T @ (_accWorld - self.accField))

        #addNoise(self.simAcc)

    def getGyroscope(self):
        _R = np.array(p.getMatrixFromQuaternion(self.simQ)).reshape(3,3)
        self.simGyro = _R.T @ np.array(self.simAngVel)

        #addNoise(self.simAcc)
    
    #probably not going to be used
    def getMagnetometer(self, magNED=np.array([0.2, 0.0, 0.5]):
        _R = np.array(p.getMatrixFromQuaternion(self.simQ)).reshape(3,3)
        self.simMag = _R.T @ magNED

        #addNoise(self.simMag)

    def getBarometer(self):
        self.simBaro = 101325 * (1 - 2.25577e-5 * self.simPos[2])**5.25588

    def sendSimSensors(self):
        self.master.hil_sensor_send(
            int(time.time()*1e6),
            self.simAcc[0], self.simAcc[1], self.simAcc[2],
            self.simGyro[0], self.simGyro[1], self.simGyro[2],
            0, 0, 0,
            self.simBaro, 0,
            self.simPos[2], 28.5,
            0xFF
            )
