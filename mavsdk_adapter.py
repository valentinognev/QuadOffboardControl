#!/bin/python3
import time
import zmq
import zmqTopics
import zmqWrapper
import pickle
import mps

import time
from common import Utils, Quaternion, Flight_Data, MAV_CMD, FLIGHT_MODE, Rate_Cmd, Logger
from low_pass_filter import Low_Pass_Filter

import numpy as np
from mavlink_parser import Mavlink_Parser
from pymavlink import mavutil
import multiprocessing
from multiprocessing import  Lock
from copy import deepcopy

from mavsdk import System
from mavsdk.offboard import (Attitude, AttitudeRate, OffboardError, VelocityNedYaw, VelocityBodyYawspeed)
import asyncio


# from config_parser import Config_Parser
import os
#sudo nano /home/user/.local/bin/mavproxy.py
#mavproxy.py --master=/dev/ttyACM0 --baudrate 57600
#find / -name "mavproxy.py" 2>/dev/null
MAVLINK_QUEUE_SIZE = 200
MAVLINK_RATE_HZ = 50
MUTEX_TIMEOUT_SEC = 1
TIME_BETWEEN_MODE_SET_ATTEMPTS = 1
ARM_LOOP_DELAY = 0.02
MAX_VERTICAL_VEL_JUMP_M_S = 3
USE_MAVLINK = True
DEVICE_STR_LIST = ["CubeBlack", "CubeOrange"]
RELEVANT_MAVLINK_MESSAGES = ['ALTITUDE', 
                             'ATTITUDE', 
                             'HIGHRES_IMU', 
                             'ATTITUDE_QUATERNION', 
                             'LOCAL_POSITION_NED', 
                             'GLOBAL_POSITION_INT', 
                             'ODOMETRY',
                             'VELOCITY_NED',
                             'ATTITUDE_TARGET', 
                             'CURRENT_MODE', 
                             'HEARTBEAT', 
                             'VFR_HUD', 
                             'DISTANCE_SENSOR', 
                             'OPTICAL_FLOW']

pubTopicsList = [
               [zmqTopics.topicMavlinkAltitude,         zmqTopics.topicMavlinkPort],
               [zmqTopics.topicMavlinkAttitude,         zmqTopics.topicMavlinkPort],
               [zmqTopics.topicMavlinkHighresIMU,       zmqTopics.topicMavlinkPort],
               [zmqTopics.topicMavlinkAttitudeQuat,     zmqTopics.topicMavlinkPort],
               [zmqTopics.topicMavlinkLocalPositionNed, zmqTopics.topicMavlinkPort],
               [zmqTopics.topicMavlinkGlobalPositionInt,zmqTopics.topicMavlinkPort],
               [zmqTopics.topicMavlinkOdometry,         zmqTopics.topicMavlinkPort],
               [zmqTopics.topicMavlinkVelocityNed,      zmqTopics.topicMavlinkPort],
               [zmqTopics.topicMavlinkAttitudeTarget,   zmqTopics.topicMavlinkPort],
               [zmqTopics.topicMavlinkCurrentMode,      zmqTopics.topicMavlinkPort],
               [zmqTopics.topicMavlinkHeartbeat,        zmqTopics.topicMavlinkPort],
               [zmqTopics.topicMavlinkVFR_HUD,          zmqTopics.topicMavlinkPort],
               [zmqTopics.topicMavlinkDistanceSensor,   zmqTopics.topicMavlinkPort],
               [zmqTopics.topicMavlinkOpticalFlow,      zmqTopics.topicMavlinkPort],
            ]

mpsDict = {}

sockPub = zmqWrapper.publisher(zmqTopics.topicMavlinkPort) #TODO: change to pubTopicsList

for topic in pubTopicsList:
    mpsDict[topic[0]] = mps.MPS(topic[0])

subSock = zmqWrapper.subscribe([zmqTopics.topicGuidenceCmdAttitude, 
                                zmqTopics.topicGuidenceCmdVelNedYaw,
                                zmqTopics.topicGuidenceCmdVelBodyYawRate,
                                zmqTopics.topicGuidenceCmdTakeoff,
                                zmqTopics.topicGuidenceCmdLand,
                                zmqTopics.topicGuidanceCmdArm,
                                ], zmqTopics.topicGuidenceCmdPort)


#################################################################################################################
#################################################################################################################
#################################################################################################################
class Hardware_Adapter():
    def __init__(self, log_dir):
        self._log_dir = log_dir
        self._prev_alt_m = None
        self._alt_vel_count = 0
        self._prev_vel_vertical = 0
        self._prev_alt_ts = time.time()
        self._vertical_speed_filter = Low_Pass_Filter(alpha=0.1, is_angle=False)
        self._altitude_filter = Low_Pass_Filter(alpha=0.3, is_angle=False) # was 0.3
        # system_config_parser = Config_Parser(path=os.path.join(config_dir, "system_config.json"))
        # vehicle_config_file_name = system_config_parser.get_value("vehicle_config_file", default_value="")
        # vehicle_data_parser = None
        # if(vehicle_config_file_name is not None):
        #     vehicle_data_parser = Config_Parser(path=os.path.join(config_dir,"vehicle", vehicle_config_file_name), save_copy=False)
        # if(vehicle_data_parser is None):
        #     print("config init failed in hardware adapter") 
        # self._connection_string = vehicle_data_parser.get_value("mavlink/connection_string", "")
        # self._baud_rate = float(vehicle_data_parser.get_value("mavlink/baud_rate", -1))
        # self._disable_offboard_control = vehicle_data_parser.get_value("control/disable_offboard_control", True)
        # if(self._disable_offboard_control):
        #     print("offboard control disabled")

        self._mavlink_parser = Mavlink_Parser()
        self._current_airspeed = 0
        self._mavlink_logger = Logger("mavlink", log_dir=self._log_dir, save_log_to_file=True, print_logs_to_console=False, datatype="TXT")

        self._offboard_control_enabled = False

#################################################################################################################
    async def run(self):
        # Start the tasks
        drone = System()
        await drone.connect(system_address='udp://:14540')
        tasks = [asyncio.ensure_future(self.attitude(drone)),
                 asyncio.ensure_future(self.highresimu(drone)),   #TODO: add highresimu to mavsdk
                 asyncio.ensure_future(self.attitudequat(drone)),
                 asyncio.ensure_future(self.odometry(drone)),
                 asyncio.ensure_future(self.positionned(drone)),
                 asyncio.ensure_future(self.globalpositionint(drone)),
                 asyncio.ensure_future(self.altitudem(drone)),
                 asyncio.ensure_future(self.listenerToCommands(drone)),
                 asyncio.ensure_future(self.status_text(drone)),
                 asyncio.ensure_future(self.velocity_ned(drone)),
                 asyncio.ensure_future(self.scaled_imu(drone)),
                 asyncio.ensure_future(self.scaled_pressure(drone)),
                 asyncio.ensure_future(self.rc_status(drone)),
                 asyncio.ensure_future(self.raw_imu(drone)),
                 ]

        await asyncio.gather(*tasks)

    async def attitude(self, drone):
        async for attitude in drone.telemetry.attitude_euler():
            data = {}
            data['ts'] = time.time()
            data['time_boot_ms'] = attitude.timestamp_us/1000.0
            data['roll'] = attitude.roll_deg
            data['pitch'] = attitude.pitch_deg
            data['yaw'] = attitude.yaw_deg

            sockPub.send_multipart([zmqTopics.topicMavlinkAttitude, pickle.dumps(data)])
    
    async def attitudeRate(self, drone):
        async for attitudeRate in drone.telemetry.attitude_angular_velocity():
            data = {}
            data['ts'] = time.time()
            data['time_boot_ms'] = attitudeRate.timestamp_us/1000.0
            data['roll'] = attitudeRate.roll_deg
            data['pitch'] = attitudeRate.pitch_deg
            data['yaw'] = attitudeRate.yaw_deg
            data['thrust'] = attitudeRate.thrust_percentage

            sockPub.send_multipart([zmqTopics.topicMavlinkAttitude, pickle.dumps(data)])

    async def highresimu(self, drone):
        async for imu in drone.telemetry.imu():
            data = {}
            data['ts'] = time.time()
            data['time_boot_ms'] = imu.timestamp_us/1000.0
            data['xacc'] = imu.acceleration_frd.forward_m_s2
            data['yacc'] = imu.acceleration_frd.right_m_s2
            data['zacc'] = imu.acceleration_frd.down_m_s2
            data['xgyro'] = imu.angular_velocity_frd.forward_rad_s
            data['ygyro'] = imu.angular_velocity_frd.right_rad_s
            data['zgyro'] = imu.angular_velocity_frd.down_rad_s
            data['xmag'] = imu.magnetic_field_frd.forward_gauss
            data['ymag'] = imu.magnetic_field_frd.right_gauss
            data['zmag'] = imu.magnetic_field_frd.down_gauss
            data['temperature'] = imu.temperature_degc
            
            sockPub.send_multipart([zmqTopics.topicMavlinkHighresIMU, pickle.dumps(data)])

    async def attitudequat(self, drone):
        async for attitudequat in drone.telemetry.attitude_quaternion():
            data = {}
            data['ts'] = time.time()
            data['time_boot_ms'] = attitudequat.timestamp_us/1000.0
            data['q1'] = attitudequat.w
            data['q2'] = attitudequat.x
            data['q3'] = attitudequat.y
            data['q4'] = attitudequat.z

            sockPub.send_multipart([zmqTopics.topicMavlinkAttitudeQuat, pickle.dumps(data)])

    async def odometry(self, drone):
        async for odometry in drone.telemetry.odometry():
            data = {}
            data['ts'] = time.time()
            data['roll_rad_s'] = odometry.angular_velocity_body.roll_rad_s
            data['pitch_rad_s'] = odometry.angular_velocity_body.pitch_rad_s
            data['yaw_rad_s'] = odometry.angular_velocity_body.yaw_rad_s           
            # data['pose_cov_mat'] = odometry.pose_covariance.covariance_matrix            
            data['x_m'] = odometry.position_body.x_m
            data['y_m'] = odometry.position_body.y_m
            data['z_m'] = odometry.position_body.z_m
            data['q1'] = odometry.q.w
            data['q2'] = odometry.q.x
            data['q3'] = odometry.q.y
            data['q4'] = odometry.q.z
            data['time_boot_ms'] = odometry.time_usec/1000.0
            data['vel_body_x_m_s'] = odometry.velocity_body.x_m_s
            data['vel_body_y_m_s'] = odometry.velocity_body.y_m_s
            data['vel_body_z_m_s'] = odometry.velocity_body.z_m_s
            # data['velocity_covariance'] = odometry.velocity_covariance.covariance_matrix
            data['frame_id'] = odometry.frame_id.name
            data['child_frame_id'] = odometry.child_frame_id.name
            
            sockPub.send_multipart([zmqTopics.topicMavlinkOdometry, pickle.dumps(data)])

    async def positionned(self, drone):
        async for positionned in drone.telemetry.position_velocity_ned():
            data = {}
            data['ts'] = time.time()
            data['north_m'] = positionned.position.north_m
            data['east_m'] = positionned.position.east_m
            data['down_m'] = positionned.position.down_m
            data['vx_m_s'] = positionned.velocity.north_m_s
            data['vy_m_s'] = positionned.velocity.east_m_s
            data['vz_m_s'] = positionned.velocity.down_m_s
            sockPub.send_multipart([zmqTopics.topicMavlinkLocalPositionNed, pickle.dumps(data)])

    async def globalpositionint(self, drone):
        async for positionlla in drone.telemetry.position():
            data = {}
            data['ts'] = time.time()
            data['lat'] = positionlla.latitude_deg
            data['lon'] = positionlla.longitude_deg
            data['alt'] = positionlla.absolute_altitude_m
            data['relative_alt'] = positionlla.relative_altitude_m
            sockPub.send_multipart([zmqTopics.topicMavlinkGlobalPositionInt, pickle.dumps(data)])

    async def altitudem(self, drone):
        async for altitudem in drone.telemetry.altitude():
            data = {}
            data['ts'] = time.time()
            data['amsl_m'] = altitudem.altitude_amsl_m
            data['local_m'] = altitudem.altitude_local_m
            data['monotonic_m'] = altitudem.altitude_monotonic_m
            data['relative_m'] = altitudem.altitude_relative_m
            data['terrain_m'] = altitudem.altitude_terrain_m
            data['bottom_clearance_m'] = altitudem.bottom_clearance_m

            sockPub.send_multipart([zmqTopics.topicMavlinkAltitude, pickle.dumps(data)])
                        
    async def status_text(self, drone):
        async for status_text in drone.telemetry.status_text():
            data = {}
            data['ts'] = time.time()
            data['text'] = status_text.text
     
    async def velocity_ned(self, drone):
        async for velocity_ned in drone.telemetry.velocity_ned():
            data = {}
            data['ts'] = time.time()
            data['vx_m_s'] = velocity_ned.north_m_s
            data['vy_m_s'] = velocity_ned.east_m_s
            data['vz_m_s'] = velocity_ned.down_m_s

    async def scaled_imu(self, drone):
        async for scaled_imu in drone.telemetry.scaled_imu():
            data = {}
            data['ts'] = time.time()
            data['xacc'] = scaled_imu.acceleration_frd.forward_m_s2
            data['yacc'] = scaled_imu.acceleration_frd.right_m_s2
            data['zacc'] = scaled_imu.acceleration_frd.down_m_s2
            
    async def scaled_pressure(self, drone):
        async for scaled_pressure in drone.telemetry.scaled_pressure():
            data = {}
            data['ts'] = time.time()
            data['time_boot_ms'] = scaled_pressure.timestamp_us/1000.0
            data['absolute_press_hpa'] = scaled_pressure.absolute_pressure_hpa
            data['diff_press_hpa'] = scaled_pressure.differential_pressure_hpa
            data['temperature_deg'] = scaled_pressure.temperature_deg

    async def rc_status(self, drone):
        async for rc_status in drone.telemetry.rc_status():
            data = {}
            data['is_available'] = rc_status.is_available
            data['signal_strength_percent'] = rc_status.signal_strength_percent
            data['was_available_once'] = rc_status.was_available_once

    async def raw_imu(self, drone):
        async for raw_imu in drone.telemetry.raw_imu():
            data = {}
            data['ts'] = time.time()
            data['xacc'] = raw_imu.acceleration_frd.forward_m_s2
            data['yacc'] = raw_imu.acceleration_frd.right_m_s2
            data['zacc'] = raw_imu.acceleration_frd.down_m_s2
            
    async def raw_gps(self, drone):
        async for raw_gps in drone.telemetry.raw_gps():
            data = {}
            data['ts'] = time.time()
            data['lat'] = raw_gps.latitude_deg
            data['lon'] = raw_gps.longitude_deg
            data['alt'] = raw_gps.altitude_m
            
    async def heading(self, drone):
        async for heading in drone.telemetry.heading():
            data = {}
            data['ts'] = time.time()
            data['heading_deg'] = heading.heading_deg
            
    async def health(self, drone):
        async for health in drone.telemetry.health():
            data = {}
            data['ts'] = time.time()
            data['is_gyrometer_calibration_ok'] = health.is_gyrometer_calibration_ok
            data['is_accelerometer_calibration_ok'] = health.is_accelerometer_calibration_ok
            data['is_magnetometer_calibration_ok'] = health.is_magnetometer_calibration_ok

    async def flight_mode(self, drone):
        async for flight_mode in drone.telemetry.flight_mode():
            data = {}
            data['ts'] = time.time()
            data['flight_mode'] = flight_mode.flight_mode
            
    async def ground_truth(self, drone):
        async for ground_truth in drone.telemetry.ground_truth():
            data = {}
            data['ts'] = time.time()
            data['lat'] = ground_truth.latitude_deg
            data['lon'] = ground_truth.longitude_deg
            data['alt'] = ground_truth.altitude_m
    
    async def in_air(self, drone):
        async for in_air in drone.telemetry.in_air():
            data = {}
            data['ts'] = time.time()
            data['in_air'] = in_air.in_air
            
    async def landing_state(self, drone):
        async for landing_state in drone.telemetry.landing_state():
            data = {}
            data['ts'] = time.time()
            data['landing_state'] = landing_state.landing_state
            
################################################################################################################
    async def listenerToCommands(self, drone):
        while True:
            ret = zmq.select([subSock], [], [], timeout=0.001)
            # ret = ret[0]
            if ret[0] is None or len(ret[0]) == 0:
                return
            data = subSock.recv_multipart()
            topic = data[0]
            data = pickle.loads(data[1])
            
            if topic == zmqTopics.topicGuidenceCmdAttitude:                           # mavlink ATTITUDE
                msg = pickle.loads(data[1])
                targetQuat = Quaternion(x=msg['quatNedDesBodyFrdCmd'][1], y=msg['quatNedDesBodyFrdCmd'][2], z=msg['quatNedDesBodyFrdCmd'][3], w=msg['quatNedDesBodyFrdCmd'][0])
                rpyRateCmd = Rate_Cmd(roll=msg['rpyRateCmd'][0], pitch=msg['rpyRateCmd'][1], yaw=msg['rpyRateCmd'][2])
                thrustCmd = msg['thrustCmd']
                targetRPY = targetQuat.to_euler()
                isRate = data['isRate']
                if isRate:
                    await drone.offboard.set_attitude(Attitude(roll_deg=targetRPY.roll, 
                                                            pitch_deg=targetRPY.pitch, 
                                                            yaw_deg=targetRPY.yaw, 
                                                            thrust_value=thrustCmd))
                else:
                    await drone.offboard.set_attitude_rate(AttitudeRate(roll_deg_s=rpyRateCmd.roll, 
                                                                pitch_deg_s=rpyRateCmd.pitch, 
                                                                yaw_deg_s=rpyRateCmd.yaw, 
                                                                thrust_value=thrustCmd))
                
            elif topic == zmqTopics.topicGuidenceCmdVelNedYaw:     
                yawCmd = data['yawCmd']
                velCmd = data['velCmd']
                await drone.offboard.set_velocity_ned(VelocityNedYaw(north_m_s=velCmd[0], east_m_s=velCmd[1], down_m_s=velCmd[2], yaw_deg=yawCmd))
                
            elif topic == zmqTopics.topicGuidenceCmdVelBodyYawRate:     
                yawRateCmd = data['yawRateCmd']
                velCmd = data['velCmd']
                await drone.offboard.set_velocity_body(VelocityBodyYawspeed(forward_m_s=velCmd[0], right_m_s=velCmd[1], down_m_s=velCmd[2], yawspeed_deg_s=yawRateCmd))
                
            # elif topic == zmqTopics.topicGuidenceCmdAccYaw:
            #     yawCmd = data['yawCmd']
            #     accCmd = data['accCmd']
            #     self._send_setpoint(pos=None, vel=None, acc=accCmd, yaw=yawCmd, yaw_rate=None)
                
            # elif topic == zmqTopics.topicGuidenceCmdTakeoff:
            #     msg = pickle.loads(data[1])
            #     takeoff_altitude = msg['takeoff_altitude']
            #     self._send_takeoff_cmd(takeoff_altitude)
                
            # elif topic == zmqTopics.topicGuidenceCmdLand:
            #     self._send_land_cmd()
                
            # elif topic == zmqTopics.topicGuidanceCmdArm:
            #     msg = pickle.loads(data[1])
            #     self._arm()
                    
        
#################################################################################################################

    def _send_takeoff_cmd(self, takeoff_altitude = 10):
        if(self._mavlink_connected_to_usb):
            print("when connected to real flight controller takeoff is currently not allowed")
            return
        if(self._disable_offboard_control):
            return 
 
        print("Connected to PX4 autopilot")
        print(self.mavsdk.mode_mapping())
        mode_id = self.mavsdk.mode_mapping()["TAKEOFF"][1]
        print(mode_id)
        msg = self.mavsdk.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        starting_alt = msg.alt / 1000
        takeoff_params = [0, 0, 0, 0, float("NAN"), float("NAN"), starting_alt + takeoff_altitude]
        time.sleep(1)
        # Change mode to takeoff (PX4)
        self.mavsdk.mav.command_long_send(self.mavsdk.target_system,
            self.mavsdk.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                    0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
        ack_msg = self.mavsdk.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        print(f"Change Mode ACK:  {ack_msg}")
        time.sleep(1)
 
         # Command Takeoff
        self.mavsdk.mav.command_long_send(
            self.mavsdk.target_system,
            self.mavsdk.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, takeoff_params[0], takeoff_params[1], takeoff_params[2], takeoff_params[3], takeoff_params[4], takeoff_params[5], takeoff_params[6])
        
        time.sleep(1)
       # Arm the UAS
        self.mavsdk.mav.command_long_send(self.mavsdk.target_system,
            self.mavsdk.target_component,
                                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        arm_msg = self.mavsdk.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        print(f"Arm ACK:  {arm_msg}")

        time.sleep(5)
        print("Takeoff done")
#################################################################################################################
    def _send_arm_cmd(self):
        if(self._mavlink_connected_to_usb):
            print("when connected to real flight controller takeoff is currently not allowed")
            return
        if(self._disable_offboard_control):
            return 
 
        print("Connected to PX4 autopilot")
        print(self.mavsdk.mode_mapping())
        mode_id = self.mavsdk.mode_mapping()["TAKEOFF"][1]
        print(mode_id)
        msg = self.mavsdk.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        starting_alt = msg.alt / 1000
        takeoff_params = [0, 0, 0, 0, float("NAN"), float("NAN"), starting_alt + 1]
        time.sleep(1)
        # Change mode to takeoff (PX4)
        self.mavsdk.mav.command_long_send(self.mavsdk.target_system,
            self.mavsdk.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                    0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
        ack_msg = self.mavsdk.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        print(f"Change Mode ACK:  {ack_msg}")
        time.sleep(1)
 
        #  Command Takeoff
        self.mavsdk.mav.command_long_send(
            self.mavsdk.target_system,
            self.mavsdk.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, takeoff_params[0], takeoff_params[1], takeoff_params[2], takeoff_params[3], takeoff_params[4], takeoff_params[5], takeoff_params[6])
        
        time.sleep(1)
       # Arm the UAS
        self.mavsdk.mav.command_long_send(self.mavsdk.target_system,
            self.mavsdk.target_component,
                                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        arm_msg = self.mavsdk.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        print(f"Arm ACK:  {arm_msg}")

        time.sleep(1)
        print("Takeoff done")
   
#################################################################################################################
    def _filter_data(self, current_mavlink_data:Flight_Data):
        vel_vertical = 0
        # current_mavlink_data.altitude_m.relative = self._vertical_speed_filter.step(current_mavlink_data.altitude_m.relative)
        if(self._prev_alt_m is not None):
            
            dt = current_mavlink_data.altitude_m.timestamp - self._prev_alt_ts
            if(dt != 0 ):
                current_mavlink_data.altitude_m.relative = self._altitude_filter.step(current_mavlink_data.altitude_m.relative)
                d_alt = current_mavlink_data.altitude_m.relative - self._prev_alt_m
                vel_vertical = d_alt/dt
                if(abs(vel_vertical - self._prev_vel_vertical) > MAX_VERTICAL_VEL_JUMP_M_S): #10, 0
                    # print("vel jump",abs(vel_vertical - self._prev_vel_vertical))
                    if(vel_vertical - self._prev_vel_vertical > 0 ):
                        vel_vertical = self._prev_vel_vertical + MAX_VERTICAL_VEL_JUMP_M_S
                    else:
                        vel_vertical = self._prev_vel_vertical - MAX_VERTICAL_VEL_JUMP_M_S
                
                current_mavlink_data.altitude_m.vertical_speed_estimate = self._vertical_speed_filter.step(vel_vertical)
                self._prev_vel_vertical = current_mavlink_data.altitude_m.vertical_speed_estimate
                if(self._alt_vel_count < 10 and 0):
                    self._alt_vel_count = self._alt_vel_count + 1
                    current_mavlink_data.altitude_m.vertical_speed_estimate  = 0
                # else:
                    # d_alt = current_mavlink_data.altitude_m.relative - self._prev_alt_m
                    # vel_vertical = d_alt/dt
                    # current_mavlink_data.altitude_m.vertical_speed_estimate = self._vertical_speed_filter.step(vel_vertical)
                # print("vel vertical",current_mavlink_data.altitude_m.vertical_speed_estimate, d_alt, dt, current_mavlink_data.altitude_m.relative)
                
                self._prev_alt_m = np.copy(current_mavlink_data.altitude_m.relative)
                self._prev_alt_ts = np.copy(current_mavlink_data.altitude_m.timestamp)
        else:
            self._prev_alt_m = np.copy(current_mavlink_data.altitude_m.relative)
            self._prev_alt_ts = np.copy(current_mavlink_data.altitude_m.timestamp)
            
#################################################################################################################
    def _update_data(self, new_data_dict, current_data):
        if(new_data_dict is None):
            return         

        if 'topic' in new_data_dict.keys():
            try:
                new_data = self._mavlink_parser.parse(new_data_dict, current_data)
            except Exception as e:
                try:
                    new_data = self._mavlink_parser.parse(new_data_dict, current_data)
                    print("error parsing mavlink",e)
                except Exception as e:
                    pass
            else:
                pass
        else:
            data = pickle.loads(new_data_dict[1])
        pass
                        # print(e)
#################################################################################################################
    def _send_setpoint(self, pos=None, vel=None, acc=None, yaw=None, yaw_rate=None):
        # if(not self._offboard_control_enabled):
        #     return
        time_boot_ms = 0
        coordinate_frame =mavutil.mavlink.MAV_FRAME_LOCAL_NED
        target_system = self.mavsdk.target_system
        target_component = self.mavsdk.target_component
        # Bitmask to indicate which fields should be ignored by the vehicle 
        # (see POSITION_TARGET_TYPEMASK enum) https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
        # bit1:PosX, bit2:PosY, bit3:PosZ, bit4:VelX, bit5:VelY, bit6:VelZ, 
        # bit7:AccX, bit8:AccY, bit9:AccZ, bit10:Force(if1) or bit11:yaw, bit12:yaw rate
        # When providing Pos, Vel and/or Accel all 3 axis must be provided. 
        # At least one of Pos, Vel and Accel must be provided (e.g. providing 
        # Yaw or YawRate alone is not supported)                    
        # Use Position : 0b110111111000 / 0x0DF8 / 3576 (decimal)
        # Use Velocity : 0b110111000111 / 0x0DC7 / 3527 (decimal)
        # Use Acceleration : 0b110000111111 / 0x0C3F / 3135 (decimal)
        # Use Force : 0b111000111111 
        # Use Pos+Vel : 0b110111000000 / 0x0DC0 / 3520 (decimal)
        # Use Pos+Vel+Accel : 0b110000000000 / 0x0C00 / 3072 (decimal)
        # Use Yaw : 0b100111111111 / 0x09FF / 2559 (decimal)
        # Use Yaw Rate : 0b010111111111 / 0x05FF / 1535 (decimal)    
        type_mask = 0b000000000000
        if pos is None:
            type_mask = type_mask | mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
            type_mask = type_mask | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
            type_mask = type_mask | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
            pos = np.zeros(3)
        if vel is None:
            type_mask = type_mask | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
            type_mask = type_mask | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
            type_mask = type_mask | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
            vel = np.zeros(3)
        if acc is None:
            type_mask = type_mask | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
            type_mask = type_mask | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
            type_mask = type_mask | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            acc = np.zeros(3)
        if yaw is None:
            type_mask = type_mask | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
            yaw = 0
        if yaw_rate is None:
            type_mask = type_mask | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
            yaw_rate = 0
        north = pos[0];        east = pos[1];        down = pos[2]
        vx = vel[0];        vy = vel[1];        vz = vel[2]
        afx = acc[0];        afy = acc[1];        afz = acc[2]

        self.mavsdk.mav.set_position_target_local_ned_send(
            time_boot_ms,
            target_system,
            target_component,
            coordinate_frame,
            type_mask,
            north,
            east,
            down,
            vx,
            vy,
            vz,
            afx,
            afy,
            afz,
            yaw,
            yaw_rate
         )

#################################################################################################################
    def _send_goal_attitude(self, goal_thrust, goal_attitude:Quaternion=None, rates:Rate_Cmd=None):
        if(not self._offboard_control_enabled):
            return
        # print("sending attitude command", goal_thrust)
        type_mask= 0b000000000000 #ignore rates
        if goal_attitude is None:
            type_mask = type_mask | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE
            goal_attitude = Quaternion.euler_to_quat(roll=0, pitch=0, yaw=0)
        if rates is None:
            type_mask = type_mask | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE
            type_mask = type_mask | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE
            type_mask = type_mask | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE
            rates = Rate_Cmd(np.zeros(3))
            
        time_boot_ms = int(time.time()*1000)
        target_system = 0
        target_component = 1
        q = [goal_attitude.w, goal_attitude.x, goal_attitude.y, goal_attitude.z]
        # q = [1,0,0,0]
        body_roll_rate = rates.rpydot[0]
        body_pitch_rate = rates.rpydot[1]
        body_yaw_rate = rates.rpydot[2]
        thrust_bodyfrd = [0,0,0]
        thrust = goal_thrust
        self.mavsdk.mav.set_attitude_target_send(
            time_boot_ms,
            self.mavsdk.target_system,
            target_component,
            type_mask,
            q,
            body_roll_rate,
            body_pitch_rate,
            body_yaw_rate,
            thrust,
            thrust_bodyfrd
        )


#################################################################################################################
    def _send_offboard_cmd(self):
        if(self._mavlink_connected_to_usb):
            print("when connected to real flight controller switching to offboard is currently not allowed")
            return
        if(self._disable_offboard_control):
            return
        mode = "OFFBOARD"
        if mode not in  self.mavsdk.mode_mapping():
            print("unknown mode")
        else:
            
            mode_id = self.mavsdk.mode_mapping()[mode]
            self.mavsdk.mav.command_long_send(
                self.mavsdk.target_system, 
                self.mavsdk.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE, 
                0,
                mode_id[0], 
                mode_id[1],
                mode_id[2], 
                0, 0, 0, 0)

#################################################################################################################
    def _arm(self):
        if(self._mavlink_connected_to_usb):
            print("when connected to real flight controller arming is currently not allowed")
            return
        if(self._disable_offboard_control):
            return 
        self.mavsdk.mav.command_long_send(
            self.mavsdk.target_system,
            self.mavsdk.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0)

#################################################################################################################
    def _send_land_cmd(self):
        self.mavsdk.mav.command_long_send(
            self.mavsdk.target_system,
            self.mavsdk.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)


if __name__ == '__main__':
    asyncio.run(Hardware_Adapter("logs").run())

    


