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
            print("connecting to main com")
            self.master = mavutil.mavlink_connection(address, baudrate)
        except:
            print("error in __init__, MAVlink refuses to connect, maybe wrong address or baudrate")
        super().__init__(**kwargs)


    def setFreq(self, nfreq):
        self.freq = nfreq

    def sendHeartbeat(self):
        print("#################################### primary com init executed ######################################\n")
        try:
            print("sending heartbeat")
            self.master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,      # or MAV_TYPE_GENERIC, used to be QUADCOPTER
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,   # still fine
                    0,                                       # base_mode
                    0,                                       # custom_mode
                    mavutil.mavlink.MAV_STATE_ACTIVE         # system_status
                    )
            self.master.wait_heartbeat(timeout=1)

            self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  
                    0,                                             
                    93,                             
                    10000,                                   
                    0, 0, 0, 0, 0                                  
                    )
        except:
            print("sending heartbeat failed :(")

        print("MAVLINK ENGAGED")

    def initSecondaryCom(self, address, baudrate):
        self.timeBoot = time.time()
        print("#################################### secondary com init executed ######################################\n")
        try:
            print("connecting to secondary com")
            self.master2 = mavutil.mavlink_connection(address, baudrate)

            print("sending heartbeat for secondary com")
            self.master2.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,      # or MAV_TYPE_GENERIC, used to be QUADCOPTER
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,   # still fine
                    0,                                       # base_mode
                    0,                                       # custom_mode
                    mavutil.mavlink.MAV_STATE_ACTIVE         # system_status
                    )
            self.master2.wait_heartbeat(timeout=1)

            self.master2.mav.command_long_send(
                    self.master2.target_system,
                    self.master2.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  
                    0,                                             
                    93,                             
                    10000,                                   
                    0, 0, 0, 0, 0                                  
                    )
        except:
            print("error in initSecondaryCom, MAVlink refuses to connect, maybe wrong address or baudrate")

    def initTertiaryCom(self, address, baudrate):
        self.timeBoot = time.time()
        print("#################################### tertiary com init executed ######################################\n")
        try:
            print("connecting to tertiary com")
            self.master3 = mavutil.mavlink_connection(address, baudrate)

            print("sending heartbeat for tertiary com")
            self.master3.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,      # or MAV_TYPE_GENERIC, used to be QUADCOPTER
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,   # still fine
                    0,                                       # base_mode
                    0,                                       # custom_mode
                    mavutil.mavlink.MAV_STATE_ACTIVE         # system_status
                    )
            self.master3.wait_heartbeat(timeout=1)

            self.master3.mav.command_long_send(
                    self.master3.target_system,
                    self.master3.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  
                    0,                                             
                    375,                             
                    10000,                                   
                    0, 0, 0, 0, 0                                  
                    )
        except:
            print("error in initTertiaryCom, MAVlink refuses to connect, maybe wrong address or baudrate")

    def setFlightmode(self, mode):
        self.master2.set_mode(mode)

        print("flight mode is set to ", mode)

    def arm(self):
        self.master2.mav.command_long_send(
                self.master2.target_system,
                self.master2.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0
                )
        print("DRONE ARMED")

    def disarm(self):
        self.master2.mav.command_long_send(
                self.master2.target_system,
                self.master2.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 0, 0, 0, 0, 0, 0, 0
                )
        print("DRONE DISARMED")

    def get(self, TYPE, block=True):
        return self.master.recv_match(type=TYPE, blocking=False)

    def sendOdometry(self, time, pos, q, vel, rotRates, cov1=[0.001]*21, cov2=[0.001]*21):
        vodom = mavlink2.MAVLink_odometry_message(
                time,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
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
        self.master2.mav.set_position_target_local_ned_send(
                time,
                self.master2.target_system,
                self.master2.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111000111,
                0, 0, 0,  #position
                vx, vy, vz,  #velocity
                0, 0, 0,  #acceleration
                0, 0  #yaw yaw_rate
                )

    def sendPlanarVelocityTarget(self, time, vx, vy, z): 
        self.master2.mav.set_position_target_local_ned_send(
                time,
                self.master2.target_system,
                self.master2.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111100011,
                0, 0, z,  #position
                vx, vy, 0,  #velocity
                0, 0, 0,  #acceleration
                0, 0  #yaw yaw_rate
                )

    def sendPositionTarget(self, time, x, y, z): 
        self.master2.mav.set_position_target_local_ned_send(
                time,
                self.master2.target_system,
                self.master2.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111111000,
                x, y, z,  #position
                0, 0, 0,  #velocity
                0, 0, 0,  #acceleration
                0, 0  #yaw yaw_rate
               )

    def sendTakeOffTarget(self, time, vz, z): 
        self.master2.mav.set_position_target_local_ned_send(
                time,
                self.master2.target_system,
                self.master2.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111011011,
                0, 0, z,  #position
                0, 0, vz,  #velocity
                0, 0, 0,  #acceleration
                0, 0  #yaw yaw_rate
               )
