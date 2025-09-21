from dataclasses import dataclass

import pybullet as p
import numpy as np
from pymavlink import mavutil

from quick_bezier import QuickBezier

import time
import pybullet_data
import math

@dataclass
class QuickBullet(QuickBezier):
    maxThrust : float = 40

    def __init__(self, address='localhost:14550', baudrate=57600, modelPath='urdf/preAlphaDrone.urdf', worldPath='plane.urdf', **kwargs):
        super().__init__(address=address, baudrate=baudrate, **kwargs)
        #self.sendHeartbeat()

        self.accField = np.array([0, 0, -9.81])

        _physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally

        p.setGravity(self.accField[0], self.accField[1], self.accField[2])
        _planeId = p.loadURDF(worldPath)
        _startPos = [0,0,0.5]
        _startOrientation = p.getQuaternionFromEuler([0,0,0])

        self.object = p.loadURDF(modelPath, _startPos, _startOrientation)

        p.resetBasePositionAndOrientation(self.object, _startPos, _startOrientation)
        
        self.initSimState()

        print("simulation initialisation is done successfully\n")

    def initSimState(self):
        self.simPos, self.simQ = p.getBasePositionAndOrientation(self.object)
        self.simVel, self.simAngVel = p.getBaseVelocity(self.object)

        self.simPosP, self.simQP = p.getBasePositionAndOrientation(self.object)
        self.simVelP, self.simAngVelP = p.getBaseVelocity(self.object)

        self.timeC = time.time()
        self.timeP = time.time()
        self.dt = 0.1
        
        self.propellerJoints = [0, 1, 2, 3] 

        self.thrustVect = np.zeros([4,3])

    def getSimState(self):
        self.simPosP, self.simQP = self.simPos, self.simQ
        self.simVelP, self.simAngVelP = self.simVel, self.simAngVel

        self.simPos, self.simQ = p.getBasePositionAndOrientation(self.object)
        self.simVel, self.simAngVel = p.getBaseVelocity(self.object)

        self.timeC = time.time()
        self.dt = self.timeC - self.timeP
        self.timeP = self.timeC

    #overwrites takeoff() from QuickBezier
    def takeoff(self, z):
        print("attempting to take off")

        for i in range(100):
            _time = int(time.time() * 1e6) & 0xFFFFFFFF
            self.getSimState()
            self.sendPositionTarget(_time, self.pos[0], self.pos[1], z)
            time.sleep(1/self.freq)
        self.arm()
        for i in range(200):
            _time = int(time.time() * 1e6) & 0xFFFFFFFF
            self.getSimState()
            self.sendPositionTarget(_time, self.simPos[0], self.simPos[1], z)
            time.sleep(1/self.freq)

    def addNoise(self, obj, center=0.0, amplitude=0.008, dim=3):
        obj += np.random.normal(center, amplitude, dim) 

    def getAccelerometer(self):
        _R = np.array(p.getMatrixFromQuaternion(self.simQ)).reshape(3,3)
        _accWorld = (np.array(self.simVel) - np.array(self.simVelP)) / self.dt
        self.simAcc = _R.T @ (_accWorld - self.accField)

        #addNoise(self.simAcc)

    def getGyroscope(self):
        _R = np.array(p.getMatrixFromQuaternion(self.simQ)).reshape(3,3)
        self.simGyro = _R.T @ np.array(self.simAngVel)

        #addNoise(self.simAcc)
    
    #probably not going to be used
    def getMagnetometer(self, magNED=np.array([0.2, 0.0, 0.5])):
        _R = np.array(p.getMatrixFromQuaternion(self.simQ)).reshape(3,3)
        self.simMag = _R.T @ magNED

        #addNoise(self.simMag)

    def getBarometer(self):
        self.simBaro = 101325 * (1 - 2.25577e-5 * self.simPos[2])**5.25588

    def sendSimSensors(self):
        self.master.mav.hil_sensor_send(
            int(time.time()*1e6),
            self.simAcc[0], self.simAcc[1], self.simAcc[2],
            self.simGyro[0], self.simGyro[1], self.simGyro[2],
            0, 0, 0,
            self.simBaro, 0,
            self.simPos[2], 28.5,
            0xFF
            )

    def sendFakeGPS(self):
        _lat0, _lon0, _alt0 = 47.397742, 8.545594, 500

        _R = 6378137.0 
        _dlat = self.simPos[1] / _R
        _dlon = self.simPos[0] / (_R * math.cos(math.radians(_lat0)))
        _lat = _lat0 + math.degrees(_dlat)
        _lon = _lon0 + math.degrees(_dlon)
        _alt = _alt0 - self.simPos[2] 

        self.master.mav.gps_raw_int_send(
                int(time.time() * 1e6),  
                3,                       
                int(_lat * 1e7),         
                int(_lon * 1e7),         
                int(_alt * 1000),        
                100, 100, 100,           
                0, 0                     
                )

        self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_HOME,
                0,
                0,
                0,
                0,
                int(_lat * 1e7),         
                int(_lon * 1e7),         
                int(_alt * 1000),
                0
                )

    def sendFakeOdometry(self):
        _time = int(time.time() * 1e6)
        self.sendOdometry(_time, self.simPos, self.simQ, self.simVel, self.simAngVel)

    def runSimpleSensorsSim(self):
        self.getSimState()
        self.getAccelerometer()
        self.getGyroscope()
        self.getBarometer()
        self.sendFakeGPS()

        self.sendSimSensors()

    def getActuatorOutput(self):
        try:
            _actOut = self.master.recv_match(type='HIL_ACTUATOR_CONTROLS', blocking=False)
            self.actOut = np.array([_actOut.controls[0], _actOut.controls[1], _actOut.controls[2], _actOut.controls[3]])
        except:
            print("nope, no actuation")

    def actuateFakeVehicle(self):

        propeller_joints = [0, 1, 2, 3]  
        target_rpms = [2000, 2000, 2000, 2000] 
        max_torque = [5, 5, 5, 5]  

        self.thrustVect[0][2] = 4
        self.thrustVect[1][2] = 4
        self.thrustVect[2][2] = 4
        self.thrustVect[3][2] = 4

        target_velocities = [rpm * 2 * 3.1416 / 60 for rpm in target_rpms]

        p.setJointMotorControlArray(
                bodyIndex=self.object,
                jointIndices=propeller_joints,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=target_velocities,
                forces=max_torque
                )

        for i, joint in enumerate(propeller_joints):
            prop_pos, prop_orn = p.getLinkState(self.object, joint)[0:2]


            p.applyExternalForce(
                    objectUniqueId=self.object,
                    linkIndex=joint,
                    forceObj=self.thrustVect[i],
                    posObj=prop_pos,
                    flags=p.WORLD_FRAME
                    )


    def actuateVehicle(self):

        #this will be replaced by actuator output from PX4
        _target_rpms = [500, 500, 500, 500] 
        #this will be replaced accordingly
        self.maxTorque = [5, 5, 5, 5]  

        target_velocities = [rpm * 2 * 3.1416 / 60 for rpm in target_rpms]

        p.setJointMotorControlArray(
                bodyIndex=self.object,
                jointIndices=propeller_joints,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocities=target_velocities,
                forces=self.maxTorque
                )


        for _i, _joint in enumerate(self.propellerJoints):
            _pPos, _pRot = p.getLinkState(self.object, _joint)[0:2]

            #this will be replaced with a model
            #will need to rotate it based on the joint's rotation
            thrust_vector = [0, 0, 8] 

            p.applyExternalForce(
                    objectUniqueId=self.object,
                    linkIndex=_joint,
                    forceObj=thrust_vector,
                    posObj=_Pos,
                    flags=p.WORLD_FRAME
                    )

