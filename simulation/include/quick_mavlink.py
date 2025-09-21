from dataclasses import dataclass

from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2

import time

@dataclass
class QuickMav:
    freq : float = 50
    timeBoot : float = 0.0

    def __init__(self, address, baudrate, **kwargs):
        self.timeBoot = time.time()
        try:
            self.master = mavutil.mavlink_connection(address, baudrate)
        except:
            print("error in __init__, MAVlink refuses to connect, maybe wrong address or baudrate")
        super().__init__(**kwargs)


    def setFreq(self, nfreq):
        self.freq = nfreq

    def sendHeartbeat(self):
        try:
            print("sending heartbeat")
            self.master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_QUADROTOR,      # or MAV_TYPE_GENERIC
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,   # still fine
                    0,                                       # base_mode
                    0,                                       # custom_mode
                    mavutil.mavlink.MAV_STATE_ACTIVE         # system_status
                    )
            self.master.wait_heartbeat(timeout=1)
        except:
            print("sending heartbeat failed :(")

        self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  
                0,                                             
                93,                             
                10000,                                   
                0, 0, 0, 0, 0                                  
                )

        print("MAVLINK ENGAGED")


    def setFlightmode(self, mode):
        self.master.set_mode(mode)

        print("flight mode is set to ", mode)

    def arm(self):
        self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0
                )
        print("DRONE ARMED")

    def disarm(self):
        self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 0, 0, 0, 0, 0, 0, 0
                )
        print("DRONE DISARMED")

    def get(self, TYPE, block=True):
        return self.master.recv_match(type=TYPE, blocking=False)

    def setHome(self):
        #self.master.mav.command_long_send(
        #    self.master.target_system,
        #    self.master.target_component,
        #    mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        #    1, # 0 for current pos, see mavlink common.xml doc for more, good luck
        #    0.01, # roll
        #    0.01, #pitch
        #    0.1, #yaw
        #    47.3990968000 * 1e7, 
        #    8.5451335000 * 1e7, 
        #    0 * 1e3,
        #    0
        #)

        _lat_int = int(47.3990968000 * 1e7)
        _lon_int = int(8.5451335000 * 1e7) 
        _alt = int(1 * 1e3)
        _timeNow = time.time() - self.timeBoot

        self.master.mav.param_set_send(
                self.master.target_system,
                self.master.target_component,
                b"EKF2_ORIGIN_LAT",
                float(_lat_int),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32
                )

        self.master.mav.param_set_send(
                self.master.target_system,
                self.master.target_component,
                b"EKF2_ORIGIN_LON",
                float(_lon_int),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32
                )

        self.master.mav.param_set_send(
                self.master.target_system,
                self.master.target_component,
                b"EKF2_ORIGIN_ALT",
                float(_alt),
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32
                )

        self.master.mav.hil_gps_send(
                int(time.time() * 1e6)    , # Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number. [us] (type:uint64_t)
                0             , # 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix. (type:uint8_t)
                40            , # Latitude (WGS84) [degE7] (type:int32_t)
                17            , # Longitude (WGS84) [degE7] (type:int32_t)
                0             , # Altitude (MSL). Positive for up. [mm] (type:int32_t)
                0             , # GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX (type:uint16_t)
                0             , # GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX (type:uint16_t)
                0             , # GPS ground speed. If unknown, set to: 65535 [cm/s] (type:uint16_t)
                0             , # GPS velocity in north direction in earth-fixed NED frame [cm/s] (type:int16_t)
                0             , # GPS velocity in east direction in earth-fixed NED frame [cm/s] (type:int16_t)
                0             , # GPS velocity in down direction in earth-fixed NED frame [cm/s] (type:int16_t)
                65535         , # Course over ground (NOT heading, but direction of movement), 0.0..359.99 degrees. If unknown, set to: 65535 [cdeg] (type:uint16_t)
                0             , # Number of satellites visible. If unknown, set to 255 (type:uint8_t)
                0             , # GPS ID (zero indexed). Used for multiple GPS inputs (type:uint8_t)
                36000         , # Yaw of vehicle relative to Earth's North, zero means not available, use 36000 for north [cdeg] (type:uint16_t)
                )

    def sendOdometry(self, time, pos, q, vel, rotRates, cov1=[0.002]*21, cov2=[0.002]*21):
        vodom = mavlink2.MAVLink_odometry_message(
                time,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                mavutil.mavlink.MAV_FRAME_BODY_FRD,
                pos[0], pos[1], pos[2],
                [q[0], q[1], q[2], q[3]],
                vel[0], vel[1], vel[2],
                rotRates[0], rotRates[1], rotRates[2],
                cov1, 
                cov2,
                0,
                0,
                0
                )
        self.master.mav.send(vodom)

    def refeed(self):
        _translation = self.get('LOCAL_POSITION_NED', True)
        _ang = self.get('ATTITUDE', True)
        _q = self.get('ATTITUDE_QUATERNION', True)

        _time = int(time.time() * 1e6) & 0xFFFFFFFF

        self.sendOdometry(
                _time, 
                [_translation.x, _translation.y, _translation.z],
                [_q.q1, _q.q2, _q.q3, _q.q4],
                [_translation.vx, _translation.vy, _translation.vz],
                [_ang.rollspeed, _ang.pitchspeed, _ang.yawspeed]
                )

    def sendVelocityTarget(self, time, vx, vy, vz): 
        self.master.mav.set_position_target_local_ned_send(
                time,
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111000111,
                0, 0, 0,  #position
                vx, vy, vz,  #velocity
                0, 0, 0,  #acceleration
                0, 0  #yaw yaw_rate
                )

    def sendPositionTarget(self, time, x, y, z): 
        self.master.mav.set_position_target_local_ned_send(
                time,
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111111000,
                x, y, z,  #position
                0, 0, 0,  #velocity
                0, 0, 0,  #acceleration
                0, 0  #yaw yaw_rate
                )
