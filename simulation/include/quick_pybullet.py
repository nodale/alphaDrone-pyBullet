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
    maxT : float = 12.00
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
        _startPos = [0,0,0.2]

        _startOrientation = p.getQuaternionFromEuler([0,0,0])

        self.object = p.loadURDF(modelPath, _startPos, _startOrientation)

        p.resetBasePositionAndOrientation(self.object, _startPos, _startOrientation)

        self.initSimState()

        print("simulation initialisation is done successfully\n")

    def reset(self):
        base_pos = [0.0, 0.0, 0.2]
        p.resetBasePositionAndOrientation(bodyUniqueId=self.object, 
                                          posObj=base_pos,
                                          ornObj=p.getQuaternionFromEuler([0, 0, 0]))
        p.createConstraint(
            parentBodyUniqueId=self.object,
            parentLinkIndex=-1,
            childBodyUniqueId=-1,
            childLinkIndex=-1,
            jointType=p.JOINT_POINT2POINT,
            jointAxis=[0, 0, 0],
            parentFramePosition=[0, 0, 0],
            childFramePosition=base_pos,
        )


    def setupCamera(self):
        _cam_pos = [3.0, -3.0, 3.0]
        _cam_target_pos = [0.0, 0.0, 0.0]
        _up_vector = [0.0, 0.0, 0.1]

        _fov = 40.0
        _aspect = 1.78
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
        height = 720
        width = 1280

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
        #self.posP, self.qP = self.pos, self.q
        #self.rotP = self.rot
        #self.velP, self.rotRatesP = self.vel, self.rotRates

        #self.pos, self.q = p.getBasePositionAndOrientation(self.object)
        #self.rot = self.q2euler(self.q[3], self.q[0], self.q[1], self.q[2])
        ##self.q = (self.q[3], self.q[0], self.q[1], self.q[2])
        #self.vel, self.rotRates = p.getBaseVelocity(self.object)

        ##R_wb = np.array(p.getMatrixFromQuaternion(self.q)).reshape(3, 3)
        ##self.vel = R_wb.T @ self.vel

        ##change coordinate
        #self.rot = self.q2euler(self.q[3], self.q[0], -self.q[1], -self.q[2])
        #self.q = (self.q[0], self.q[1], -self.q[2], -self.q[3])
        #self.rotRates = (self.rotRates[0], -self.rotRates[1], -self.rotRates[2])
        #self.pos = (self.pos[0], -self.pos[1], -self.pos[2])
        #self.vel = (self.vel[0], -self.vel[1], -self.vel[2])

        #self.posP, self.qP = self.pos, self.q
        #self.rotP = self.rot
        #self.velP, self.rotRatesP = self.vel, self.rotRates

        #self.pos, self.q_raw = p.getBasePositionAndOrientation(self.object)  # [x,y,z,w]
        #self.vel, self.rotRates = p.getBaseVelocity(self.object)
        #self.pos = (self.pos[0], -self.pos[1], -self.pos[2])           # FLU→FRD position
        #self.vel = (self.vel[0], -self.vel[1], -self.vel[2])           # FLU→FRD velocity (world-frame)
        #self.rotRates = (self.rotRates[0], -self.rotRates[1], -self.rotRates[2])
        #x, y, z, w = self.q_raw
        #self.q = (w, x, -y, -z)                                        # Correct FRD quaternion

        self.posP, self.qP = self.pos, self.q
        self.velP, self.rotRatesP = self.vel, self.rotRates 
        self.pos, self.q_raw = p.getBasePositionAndOrientation(self.object)
        self.vel_world, self.rotRates = p.getBaseVelocity(self.object)  # WORLD FRAME
        
        self.pos = (self.pos[0], -self.pos[1], -self.pos[2])
        vel_frd_world = (self.vel_world[0], -self.vel_world[1], -self.vel_world[2])
        R_wb = np.array(p.getMatrixFromQuaternion(self.q_raw)).reshape(3, 3)
        self.vel = tuple(R_wb @ np.array(vel_frd_world)) 
        
        self.rotRates = (self.rotRates[0], -self.rotRates[1], -self.rotRates[2])
        x, y, z, w = self.q_raw
        self.q = (w, x, -y, -z)
        self.unordered_q = (x, -y, -z, w)
        self.rot = self.q2euler(self.q[0], self.q[1], self.q[2], self.q[3])

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

    def addNoise(self, obj, center=0.0, amplitude=0.2, dim=3):
        obj += np.random.normal(center, amplitude, dim) 

    def getAccelerometer(self):
        vel_now = np.array([self.vel[0], self.vel[1], self.vel[2]])
        vel_past = np.array([self.velP[0], self.velP[1], self.velP[2]])
        acc_kinematic = (vel_now - vel_past) / self.dt
        gravity_world = np.array([0, 0, -9.81])
        R_world_to_body = np.array(p.getMatrixFromQuaternion(self.unordered_q)).reshape(3, 3)
        self.simAcc = R_world_to_body.T @ gravity_world
        self.simAcc += acc_kinematic

        #_R = np.array(p.getMatrixFromQuaternion(self.q)).reshape(3,3)
        #_accWorld = (np.array(self.vel) - np.array(self.velP)) / self.dt
        #_transgravity = _R.T @ self.accField 
        #self.simAcc = _accWorld + _transgravity

        #R_wb = np.array(p.getMatrixFromQuaternion(self.q)).reshape(3, 3)
        #acc_world = (np.array(self.vel) - np.array(self.velP)) / self.dt
        #self.velP = self.vel
        #acc_body = R_wb.T @ (acc_world - np.array(self.accField))

        #self.simAcc = acc_body
        #self.simAccLPF = self.alpha * self.simAcc + (1.0 - self.alpha) * self.simAccLPF

    def getGyroscope(self):
        R_world_to_body = np.array(p.getMatrixFromQuaternion(self.unordered_q)).reshape(3, 3)
        _body_gyro = R_world_to_body.T @ np.array(self.rotRates)
        self.simGyro = _body_gyro

        self.addNoise(self.simGyro)
        #_R = np.array(p.getMatrixFromQuaternion(self.q)).reshape(3,3)
        #self.simGyro = _R.T @ np.array(self.rotRates)
        #R_wb = np.array(p.getMatrixFromQuaternion(self.q)).reshape(3, 3)
        #omega_world = np.array(self.rotRates)
        #omega_world = (omega_world[0], -omega_world[1], -omega_world[2])

        #self.simGyro = R_wb.T @ omega_world
        #self.simGyroLPF = self.alpha * self.simGyro + (1.0 - self.alpha) * self.simGyroLPF
        #self.addNoise(self.simGyro)

    #probably not going to be used
    def getMagnetometer(self, magNED=np.array([0.2, 0.0, 0.5])):
        _R = np.array(p.getMatrixFromQuaternion(self.q)).reshape(3,3)
        self.simMag = _R.T @ magNED

        #addNoise(self.simMag)

    def getBarometer(self):
        self.simBaro = 101325 * (1 - 2.25577e-5 * self.pos[2])**5.25588

    def sendSimSensors(self, time):
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
                time,
                self.simAcc[0], self.simAcc[1], self.simAcc[2],
                self.simGyro[0], self.simGyro[1], self.simGyro[2],
                0, 0, 0,
                0, 0,
                0, 0,
                0b0000000111111
                )

    def sendFakeGPS(self, time):
        _lat0, _lon0, _alt0 = 47.397742, 8.545594, 500

        _r = 6378137.0 
        _dlat = self.pos[1] / _r
        _dlon = self.pos[0] / (_r * math.cos(math.radians(_lat0)))
        _lat = _lat0 + math.degrees(_dlat)
        _lon = _lon0 + math.degrees(_dlon)
        _alt = _alt0 - self.pos[2] 

        self.master.mav.hil_gps_send(
                time,
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


    def sendFakeOdometry(self, _time):
        #_reordered_q = (self.q[3], self.q[0], self.q[1], self.q[2])
        #_reordered_pos = (-self.pos[0], -self.pos[1], self.pos[2])

        #self.addNoise(self.vel)
        #self.addNoise(self.rotRates)

        self.sendOdometry(_time, self.pos, self.q, self.vel, self.rotRates)

    def runSimpleSensorsSim(self, time):
        self.getSimState()
        self.getAccelerometer()
        self.getGyroscope()
        self.getBarometer()
        self.sendFakeGPS(time)

        self.sendSimSensors(time)

    def getActuatorOutput(self):
        try:
            _actOut = self.master.recv_match(type='HIL_ACTUATOR_CONTROLS', blocking=False)
            self.actOut = np.array([_actOut.controls[0] , _actOut.controls[1] , _actOut.controls[2] , _actOut.controls[3] ])
            #print(f"{self.actOut[0]:.2f}, {self.actOut[1]:.2f}, {self.actOut[2]:.2f}, {self.actOut[3]:.2f}")
        except:
            self.actOut = self.actOut

    def actuateFakeVehicle(self):
        x = np.hstack((self.pos, self.vel, self.rot, self.rotRates))  # state vector
        sp = np.array([
            1.0, 0.0, -0.8, 
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0
            ])

        #u_eq = np.array([8.04575222, 8.04575222, 7.55214778, 7.55214778])  
        u_eq = np.array([7.8, 7.8, 7.8, 7.8])  

        #K = np.array([
        #        [-1.144779,  1.118034, 0.616729, -2.176632,  2.113474,  1.721950, -6.434980, -6.822077, -0.070711, -1.038827, -1.133037, -0.353416],
        #        [-1.144779, -1.118034, 0.616729, -2.176632, -2.113474,  1.721950,  6.434980, -6.822077,  0.070711,  1.038827, -1.133037,  0.353416],
        #        [ 1.090084, -1.118034, 0.673570,  2.073100, -2.113474,  1.881167,  6.434980,  6.507153, -0.070711,  1.038827,  1.087294, -0.353416],
        #        [ 1.090084,  1.118034, 0.673570,  2.073100,  2.113474,  1.881167, -6.434980,  6.507153,  0.070711, -1.038827,  1.087294,  0.353416]
        #    ])

        
        K = np.array([
    [ 1.256562,  1.256562,  0.466252,  1.230240,  1.210217,  1.430276,  4.176322, -4.367069, -0.057354,  0.813507, -0.876924, -0.256517],
    [ 1.256562, -1.256562,  0.466252,  1.230240, -1.210217,  1.430276, -4.176322, -4.367069,  0.057354, -0.813507, -0.876924,  0.256517],
    [-1.256562, -1.256562,  0.466252, -1.230240, -1.210217,  1.430276, -4.176322,  4.367069, -0.057354, -0.813507,  0.876924, -0.256517],
    [-1.256562,  1.256562,  0.466252, -1.230240,  1.210217,  1.430276,  4.176322,  4.367069,  0.057354,  0.813507,  0.876924,  0.256517]
], dtype=np.float32)

        offset = (x - sp)
        #u = u_eq + K @ -offset
        u = u_eq + K @ offset
        #print(f"{offset:.3f}")
        #print(f"{offset[0]:.3f}     {offset[1]:.3f}     {offset[2]:.3f}")
        print(u)
        #print(offset)

        #u = np.clip(u, 0.0, 1.0)

        _act_sq = u
        _forces = u

        _KF = self.maxT
        _KM = 0.09 * self.maxT
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

        _spin_dir = np.array([-1, 1, -1, 1])

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

    def actuateVehicle(self):
        _act_sq = np.array(self.actOut)

        _KF = self.maxT
        _KM = 0.09 * self.maxT

        print(_act_sq)
        _forces = _act_sq * _KF
        for i in range(4):
            if _act_sq[i] <= 0.192:
                _forces[i] *= 0.0
            else:
                _forces[i] = 10.81 * ((_act_sq[i] - 0.191) / 0.689) ** (1.0 / 0.690)

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

        _spin_dir = np.array([-1, 1, -1, 1])

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
