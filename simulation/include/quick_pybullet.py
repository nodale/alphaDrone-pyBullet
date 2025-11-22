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
    maxT : float = 8.83
    text_id : None = None
    alpha : float = 0.2

    def __init__(self, address='localhost:14550', baudrate=57600, modelPath='urdf/preBetaDrone.urdf', worldPath='plane.urdf', **kwargs):
        super().__init__(address=address, baudrate=baudrate, **kwargs)

        self.accField = np.array([0, 0, -9.81])

        _physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) 

        p.setGravity(self.accField[0], self.accField[1], self.accField[2])
        _planeId = p.loadURDF(worldPath)
        _startPos = [0,0,0.4]
        _startOrientation = p.getQuaternionFromEuler([0,0,0])

        self.object = p.loadURDF(modelPath, _startPos, _startOrientation)

        p.resetBasePositionAndOrientation(self.object, _startPos, _startOrientation)

        self.initSimState()

        print("simulation initialisation is done successfully\n")

    def reset(self):
        p.resetBasePositionAndOrientation(bodyUniqueId=self.object, 
                                          posObj=[0, 0, 0.2],
                                          ornObj=p.getQuaternionFromEuler([0, 0, 0]))

    def initSimState(self):
        self.simPos, self.simQ = p.getBasePositionAndOrientation(self.object)
        #self.simQ = (self.simQ[3], self.simQ[0], self.simQ[1], self.simQ[2])
        self.simVel, self.simAngVel = p.getBaseVelocity(self.object)
        self.simRot = (0.0, 0.0, 0.0)

        self.simPosP, self.simQP = p.getBasePositionAndOrientation(self.object)
        self.simVelP, self.simAngVelP = p.getBaseVelocity(self.object)

        self.simAcc = np.zeros(3, dtype=float)
        self.simGyro = np.zeros(3, dtype=float)
        self.simAccLPF = np.zeros(3, dtype=float)
        self.simGyroLPF = np.zeros(3, dtype=float)

        self.timeC = time.time()
        self.timeP = time.time()
        self.dt = 0.1

        self.propellerJoints = [0, 1, 2, 3] 

        self.thrustVect = np.zeros([4,3])
        self.actOut = np.empty(4)


        self._temp_pos = np.empty(3)

    def getSimState(self):
        self.simPosP, self.simQP = self.simPos, self.simQ
        self.simVelP, self.simAngVelP = self.simVel, self.simAngVel

        self.simPos, self.simQ = p.getBasePositionAndOrientation(self.object)
        #self.simQ = (self.simQ[3], self.simQ[0], self.simQ[1], self.simQ[2])
        self.simVel, self.simAngVel = p.getBaseVelocity(self.object)

        R_wb = np.array(p.getMatrixFromQuaternion(self.simQ)).reshape(3, 3)
        self.simVel = R_wb.T @ self.simVel

        #change coordinate
        self.simRot = self.q2euler(self.simQ[3], self.simQ[0], -self.simQ[1], -self.simQ[2])
        self.simQ = (self.simQ[0], -self.simQ[1], -self.simQ[2], self.simQ[3])
        self.simAngVel = (self.simAngVel[0], -self.simAngVel[1], -self.simAngVel[2])
        self.simPos = (self.simPos[0], -self.simPos[1], -self.simPos[2])
        self.simVel = (self.simVel[0], -self.simVel[1], -self.simVel[2])

        self.timeC = time.time()
        #self.dt = self.timeC - self.timeP
        self.dt = 1.0/self.freq
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
        _transgravity = _R.T @ self.accField 
        self.simAcc = _accWorld + _transgravity

        #R_wb = np.array(p.getMatrixFromQuaternion(self.simQ)).reshape(3, 3)
        #acc_world = (np.array(self.simVel) - np.array(self.simVelP)) / self.dt
        #self.simVelP = self.simVel
        #acc_body = R_wb.T @ (acc_world - np.array(self.accField))

        #self.simAcc = acc_body
        #self.simAccLPF = self.alpha * self.simAcc + (1.0 - self.alpha) * self.simAccLPF
        self.addNoise(self.simAcc)

    def getGyroscope(self):
        self.simGyro = np.array(self.simAngVel)

        #_R = np.array(p.getMatrixFromQuaternion(self.simQ)).reshape(3,3)
        #self.simGyro = _R.T @ np.array(self.simAngVel)
        #R_wb = np.array(p.getMatrixFromQuaternion(self.simQ)).reshape(3, 3)
        #omega_world = np.array(self.simAngVel)
        #omega_world = (omega_world[0], -omega_world[1], -omega_world[2])

        #self.simGyro = R_wb.T @ omega_world
        #self.simGyroLPF = self.alpha * self.simGyro + (1.0 - self.alpha) * self.simGyroLPF
        self.addNoise(self.simGyro)

    #probably not going to be used
    def getMagnetometer(self, magNED=np.array([0.2, 0.0, 0.5])):
        _R = np.array(p.getMatrixFromQuaternion(self.simQ)).reshape(3,3)
        self.simMag = _R.T @ magNED

        #addNoise(self.simMag)

    def getBarometer(self):
        self.simBaro = 101325 * (1 - 2.25577e-5 * self.simPos[2])**5.25588

    def sendSimSensors(self):
        #self.master.mav.hil_sensor_send(
        #    int(time.time() * 1e6) & 0xFFFFFFFF,
        #    self.simAcc[0], self.simAcc[1], self.simAcc[2],
        #    self.simGyro[0], self.simGyro[1], self.simGyro[2],
        #    0, 0, 0,
        #    self.simBaro, 0,
        #    self.simPos[2], 28.5,
        #    0xFF
        #    )

        self.master.mav.hil_sensor_send(
                int(time.time() * 1e6) & 0xFFFFFFFF,
                self.simAcc[0], self.simAcc[1], self.simAcc[2],
                self.simGyro[0], self.simGyro[1], self.simGyro[2],
                0, 0, 0,
                0, 0,
                0, 0,
                0b0000000111111
                )

    def sendFakeGPS(self):
        _lat0, _lon0, _alt0 = 47.397742, 8.545594, 500

        _r = 6378137.0 
        _dlat = self.simPos[1] / _r
        _dlon = self.simPos[0] / (_r * math.cos(math.radians(_lat0)))
        _lat = _lat0 + math.degrees(_dlat)
        _lon = _lon0 + math.degrees(_dlon)
        _alt = _alt0 - self.simPos[2] 

        self.master.mav.hil_gps_send(
                int(time.time() * 1e6), 
                3,
                int(71 * 1e7), 
                int(-40 * 1e7), 
                int(500 * 1e3), 
                0, 
                0, 
                0, 
                0, 
                0, 
                0, 
                65535, 
                255, 
                0, 
                36000 
                )

#    def sendFakeGPS(self):
#       _lat0, _lon0, _alt0 = 47.397742, 8.545594, 500.0
#
#       _r = 6378137.0  
#
#       x, y, z = self.simPos
#
#       dlat = y / _r
#       dlon = x / (_r * math.cos(math.radians(_lat0)))
#
#       lat = _lat0 + math.degrees(dlat)
#       lon = _lon0 + math.degrees(dlon)
#       alt = _alt0 - z  
#
#       fix_type = 3 
#       eph = 100    
#       epv = 100    
#
#       self.master.mav.hil_gps_send(
#               int(time.time() * 1e6),  # timestamp (usec)
#               fix_type,                # fix type
#               int(lat * 1e7),          # latitude (degE7)
#               int(lon * 1e7),          # longitude (degE7)
#               int(alt * 1e3),          # altitude (mm)
#               int(eph),                # horizontal dilution of precision (cm)
#               int(epv),                # vertical dilution of precision (cm)
#               int(math.sqrt(self.simVel[0]**2 + self.simVel[1]**2) * 100),  # ground speed (cm/s)
#               int(math.degrees(math.atan2(self.simVel[1], self.simVel[0])) * 100),  # course over ground (cdeg)
#               int(self.simVel[2] * 100),  # vertical speed (cm/s)
#               255,  # satellites visible
#               0, 0,  # idk
#               0      # heading 
#               )

    def sendFakeOdometry(self):
        _time = int(time.time() * 1e6)
        _reordered_q = (self.simQ[3], self.simQ[0], self.simQ[1], self.simQ[2])
        #_reordered_pos = (-self.simPos[0], -self.simPos[1], self.simPos[2])
        self.sendOdometry(_time, self.simPos, _reordered_q, self.simVel, self.simAngVel)

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
            self.actOut = np.array([_actOut.controls[0] , _actOut.controls[1] , _actOut.controls[2] , _actOut.controls[3] ])
            #print(f"{self.actOut[0]:.2f}, {self.actOut[1]:.2f}, {self.actOut[2]:.2f}, {self.actOut[3]:.2f}")
        except:
            self.actOut = self.actOut

    def actuateFakeVehicle(self):

        for _i, _joint in enumerate(self.propellerJoints):
            _pPos, _pRot = p.getLinkState(self.object, _joint)[0:2]
            #this will be replaced with a model
            _temp = 4.0
            _temp = max(0.0, _temp)
            _temp *= (-1)**_i
            _thrust_vector = [0.0, 0.0, _temp] 

            p.applyExternalForce(
                    objectUniqueId=self.object,
                    linkIndex=_joint,
                    forceObj=_thrust_vector,
                    posObj=_pPos,
                    flags=p.LINK_FRAME
                    )

#    def actuateVehicle(self):
#        _act_sq = np.array(self.actOut)
#
#        _KF = self.maxT
#        _KM = 0.11 * self.maxT   
#
#        _forces = _act_sq * _KF
#        _torques = _act_sq * _KM
#
#        _arm_length = 0.158  
#        #this one is in pyBullet's coordinate sys
#        _positions = np.array([
#            [ _arm_length, -_arm_length, 0],  
#            [ _arm_length, _arm_length, 0],  
#            [-_arm_length, _arm_length, 0],  
#            [-_arm_length, -_arm_length, 0],  
#            ])
#
#        _spin_dir = np.array([1, -1, 1, -1])
#
#        _total_force = np.zeros(3)
#        _total_torque = np.zeros(3)
#
#        for i in range(4):
#            f_i = np.array([0, 0, _forces[i]])
#
#            tau_z = np.array([0, 0, _spin_dir[i] * _torques[i]])
#
#            tau_arm = np.cross(_positions[i], f_i)
#
#            _total_force += f_i
#            _total_torque += tau_arm + tau_z
#
#        _, quat = p.getBasePositionAndOrientation(self.object)
#        R_wb = np.array(p.getMatrixFromQuaternion(quat)).reshape(3, 3)
#        _total_force_body = R_wb.T @ _total_force
#        _total_torque_body = R_wb.T @ _total_torque
#
#        p.applyExternalForce(
#            self.object, -1,
#            forceObj=_total_force_body.tolist(),
#            posObj=[0, 0, 0],
#            flags=p.LINK_FRAME,
#        )
#
#        p.applyExternalTorque(
#            self.object, -1,
#            torqueObj=_total_torque_body.tolist(),
#            flags=p.LINK_FRAME,
#        )

    def actuateVehicle(self):
        _act_sq = np.array(self.actOut)

        _KF = self.maxT
        _KM = 0.09 * self.maxT

        _forces = _act_sq * _KF
        _torques = _act_sq * _KM

        _arm_length = 0.158
        _x_offset = 0.00323
        _y_offset = 0.001
        _z_offset = -0.014
        _positions = np.array([
            [ _arm_length - _x_offset, -_arm_length - _y_offset, 0.0 - _z_offset],   
            [ _arm_length - _x_offset,  _arm_length - _y_offset, 0.0 - _z_offset],  
            [-_arm_length - _x_offset,  _arm_length - _y_offset, 0.0 - _z_offset], 
            [-_arm_length - _x_offset, -_arm_length - _y_offset, 0.0 - _z_offset], 
        ])

        _spin_dir = np.array([1, -1, 1, -1])

        _, quat = p.getBasePositionAndOrientation(self.object)
        R_wb = np.array(p.getMatrixFromQuaternion(quat)).reshape(3, 3)

        for i in range(4):
            f_body = np.array([0, 0, _forces[i]])

            tau_body = np.array([0, 0, _spin_dir[i] * _torques[i]])

            p.applyExternalForce(
                self.object, -1,
                forceObj=f_body.tolist(),
                posObj=_positions[i].tolist(),
                flags=p.LINK_FRAME,
            )

            p.applyExternalTorque(
                self.object, -1,
                torqueObj=tau_body.tolist(),
                flags=p.LINK_FRAME,
            )

    def q2euler(self, w, x, y, z):
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return (roll, pitch, yaw)

    def showState(self):
        _text = (
                f"Pos [m]:     x={self.simPos[0]:+.3f}, y={self.simPos[1]:+.3f}, z={self.simPos[2]:+.3f}\n"
                f"Euler [deg]: roll={self.simRot[0]:+.1f}, pitch={self.simRot[1]:+.1f}, yaw={self.simRot[2]:+.1f}\n"
                f"Lin vel [m/s]: vx={self.simVel[0]:+.3f}, vy={self.simVel[1]:+.3f}, vz={self.simVel[2]:+.3f}\n"
                f"Ang vel [rad/s]: wx={self.simAngVel[0]:+.3f}, wy={self.simAngVel[1]:+.3f}, wz={self.simAngVel[2]:+.3f}"
                )

        # --- Remove old text and add new one ---
        if self.text_id is not None:
            p.removeUserDebugItem(self.text_id)

        self.text_id = p.addUserDebugText(
                _text, [0.2, 0.2, 1.5], textColorRGB=[0, 0, 0], textSize=1.2, lifeTime=0
                )
