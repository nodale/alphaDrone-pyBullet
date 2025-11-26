from dataclasses import dataclass

import pybullet as p
import numpy as np
from pymavlink import mavutil
from PIL import Image

from quick_bezier import QuickBezier

import time
import pybullet_data
import math

@dataclass
class QuickBullet(QuickBezier):
    maxT : float = 8.83
    text_id : None = None
    alpha : float = 0.2
    timestamp : float = 0.0

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

    def setupCamera(self):
        _cam_pos = [1.0, 1.0, 1.0]
        _cam_target_pos = [0.0, 0.0, 0.0]
        _up_vector = [0.0, 0.0, 0.1]

        _fov = 60.0
        _aspect = 1.0
        _near = 0.1
        _far = 10.0

        self.view_matrix = p.computeViewMatrix(
                _cam_pos,
                _cam_target_pos,
                _up_vector
                )

        self.proj_matrix = p.computeProjectionMatrixFOV(
                _fov,
                _aspect,
                _near,
                _far
                )

    def printCamera(self):
        height = 200
        width = 200

        img = p.getCameraImage(
                width=width,
                height=height,
                viewMatrix=self.view_matrix,
                projectionMatrix=self.proj_matrix,
                renderer=p.ER_BULLET_HARDWARE_OPENGL
                )
        
        rgba = np.reshape(img[2], (height, width, 4)) 
        rgb = rgba[:, :, :3]

        rgb = np.flip(rgb, axis=0).astype(np.uint8)

        pic = Image.fromarray(rgb, mode='RGB')
        pic = pic.transpose(Image.FLIP_TOP_BOTTOM)
        _time = int(self.timestamp * 1e6) & 0xFFFFFFFF
        pic.save(f"recording/cam_{_time}.png")
        

    def initSimState(self):
        self.pos, self.q = p.getBasePositionAndOrientation(self.object)
        #self.q = (self.q[3], self.q[0], self.q[1], self.q[2])
        self.vel, self.rotRates = p.getBaseVelocity(self.object)
        self.rot = (0.0, 0.0, 0.0)

        self.posP, self.qP = p.getBasePositionAndOrientation(self.object)
        self.velP, self.rotRatesP = p.getBaseVelocity(self.object)

        self.simAcc = np.zeros(3, dtype=float)
        self.simGyro = np.zeros(3, dtype=float)
        self.simAccLPF = np.zeros(3, dtype=float)
        self.simGyroLPF = np.zeros(3, dtype=float)

        self.timeC = self.timestamp
        self.timeP = self.timestamp
        self.dt = 0.1

        self.propellerJoints = [0, 1, 2, 3] 

        self.thrustVect = np.zeros([4,3])
        self.actOut = np.empty(4)


        self._temp_pos = np.empty(3)

    def getSimState(self):
        self.posP, self.qP = self.pos, self.q
        self.velP, self.rotRatesP = self.vel, self.rotRates

        self.pos, self.q = p.getBasePositionAndOrientation(self.object)
        #self.q = (self.q[3], self.q[0], self.q[1], self.q[2])
        self.vel, self.rotRates = p.getBaseVelocity(self.object)

        R_wb = np.array(p.getMatrixFromQuaternion(self.q)).reshape(3, 3)
        self.vel = R_wb.T @ self.vel

        #change coordinate
        self.rot = self.q2euler(self.q[3], self.q[0], -self.q[1], -self.q[2])
        self.q = (self.q[0], -self.q[1], -self.q[2], self.q[3])
        self.rotRates = (self.rotRates[0], -self.rotRates[1], -self.rotRates[2])
        self.pos = (self.pos[0], -self.pos[1], -self.pos[2])
        self.vel = (self.vel[0], -self.vel[1], -self.vel[2])

        self.timeC = time.time()
        #self.dt = self.timeC - self.timeP
        self.dt = 1.0/self.freq
        self.timestamp += self.dt
        self.timeP = self.timeC

    #overwrites takeoff() from QuickBezier
    def takeoff(self, z):
        print("attempting to take off")

        for i in range(100):
            _time = int(self.timestamp * 1e6) & 0xFFFFFFFF
            self.getSimState()
            self.sendPositionTarget(_time, self.pos[0], self.pos[1], z)
            time.sleep(1/self.freq)
        self.arm()
        for i in range(200):
            _time = int(self.timestamp * 1e6) & 0xFFFFFFFF
            self.getSimState()
            self.sendPositionTarget(_time, self.pos[0], self.pos[1], z)
            time.sleep(1/self.freq)

    def addNoise(self, obj, center=0.0, amplitude=0.01, dim=3):
        obj += np.random.normal(center, amplitude, dim) 

    def getAccelerometer(self):
        _R = np.array(p.getMatrixFromQuaternion(self.q)).reshape(3,3)
        _accWorld = (np.array(self.vel) - np.array(self.velP)) / self.dt
        _transgravity = _R.T @ self.accField 
        self.simAcc = _accWorld + _transgravity

        #R_wb = np.array(p.getMatrixFromQuaternion(self.q)).reshape(3, 3)
        #acc_world = (np.array(self.vel) - np.array(self.velP)) / self.dt
        #self.velP = self.vel
        #acc_body = R_wb.T @ (acc_world - np.array(self.accField))

        #self.simAcc = acc_body
        #self.simAccLPF = self.alpha * self.simAcc + (1.0 - self.alpha) * self.simAccLPF
        self.addNoise(self.simAcc)

    def getGyroscope(self):
        self.simGyro = np.array(self.rotRates)

        #_R = np.array(p.getMatrixFromQuaternion(self.q)).reshape(3,3)
        #self.simGyro = _R.T @ np.array(self.rotRates)
        #R_wb = np.array(p.getMatrixFromQuaternion(self.q)).reshape(3, 3)
        #omega_world = np.array(self.rotRates)
        #omega_world = (omega_world[0], -omega_world[1], -omega_world[2])

        #self.simGyro = R_wb.T @ omega_world
        #self.simGyroLPF = self.alpha * self.simGyro + (1.0 - self.alpha) * self.simGyroLPF
        self.addNoise(self.simGyro)

    #probably not going to be used
    def getMagnetometer(self, magNED=np.array([0.2, 0.0, 0.5])):
        _R = np.array(p.getMatrixFromQuaternion(self.q)).reshape(3,3)
        self.simMag = _R.T @ magNED

        #addNoise(self.simMag)

    def getBarometer(self):
        self.simBaro = 101325 * (1 - 2.25577e-5 * self.pos[2])**5.25588

    def sendSimSensors(self):
        #self.master.mav.hil_sensor_send(
        #    int(self.timestamp * 1e6) & 0xFFFFFFFF,
        #    self.simAcc[0], self.simAcc[1], self.simAcc[2],
        #    self.simGyro[0], self.simGyro[1], self.simGyro[2],
        #    0, 0, 0,
        #    self.simBaro, 0,
        #    self.pos[2], 28.5,
        #    0xFF
        #    )

        self.master.mav.hil_sensor_send(
                int(self.timestamp * 1e6) & 0xFFFFFFFF,
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
        _dlat = self.pos[1] / _r
        _dlon = self.pos[0] / (_r * math.cos(math.radians(_lat0)))
        _lat = _lat0 + math.degrees(_dlat)
        _lon = _lon0 + math.degrees(_dlon)
        _alt = _alt0 - self.pos[2] 

        self.master.mav.hil_gps_send(
                int(self.timestamp * 1e6), 
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
#       x, y, z = self.pos
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
#               int(self.timestamp * 1e6),  # timestamp (usec)
#               fix_type,                # fix type
#               int(lat * 1e7),          # latitude (degE7)
#               int(lon * 1e7),          # longitude (degE7)
#               int(alt * 1e3),          # altitude (mm)
#               int(eph),                # horizontal dilution of precision (cm)
#               int(epv),                # vertical dilution of precision (cm)
#               int(math.sqrt(self.vel[0]**2 + self.vel[1]**2) * 100),  # ground speed (cm/s)
#               int(math.degrees(math.atan2(self.vel[1], self.vel[0])) * 100),  # course over ground (cdeg)
#               int(self.vel[2] * 100),  # vertical speed (cm/s)
#               255,  # satellites visible
#               0, 0,  # idk
#               0      # heading 
#               )

    def sendFakeOdometry(self):
        _time = int(self.timestamp * 1e6)
        _reordered_q = (self.q[3], self.q[0], self.q[1], self.q[2])
        #_reordered_pos = (-self.pos[0], -self.pos[1], self.pos[2])

        self.addNoise(self.vel)
        self.addNoise(self.rotRates)

        self.sendOdometry(_time, self.pos, _reordered_q, self.vel, self.rotRates)

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
                f"Pos [m]:     x={self.pos[0]:+.3f}, y={self.pos[1]:+.3f}, z={self.pos[2]:+.3f}\n"
                f"Euler [deg]: roll={self.rot[0]:+.1f}, pitch={self.rot[1]:+.1f}, yaw={self.rot[2]:+.1f}\n"
                f"Lin vel [m/s]: vx={self.vel[0]:+.3f}, vy={self.vel[1]:+.3f}, vz={self.vel[2]:+.3f}\n"
                f"Ang vel [rad/s]: wx={self.rotRates[0]:+.3f}, wy={self.rotRates[1]:+.3f}, wz={self.rotRates[2]:+.3f}"
                )

        # --- Remove old text and add new one ---
        if self.text_id is not None:
            p.removeUserDebugItem(self.text_id)

        self.text_id = p.addUserDebugText(
                _text, [0.2, 0.2, 1.5], textColorRGB=[0, 0, 0], textSize=1.2, lifeTime=0
                )
