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

from common import Flight_Data
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
pubTopicsList = [
               [zmqTopics.topicMavlinkFlightData,         zmqTopics.topicMavlinkPort],
            ]

mpsDict = {}

sockPub = zmqWrapper.publisher(zmqTopics.topicMavlinkPort) #TODO: change to pubTopicsList

for topic in pubTopicsList:
    mpsDict[topic[0]] = mps.MPS(topic[0])

subsList = [zmqTopics.topicGuidenceCmdAttitude, 
                                zmqTopics.topicGuidenceCmdVelNedYaw,
                                zmqTopics.topicGuidenceCmdVelBodyYawRate,
                                zmqTopics.topicGuidenceCmdTakeoff,
                                zmqTopics.topicGuidenceCmdLand,
                                zmqTopics.topicGuidanceCmdArm,
                                ]
subsPort = zmqTopics.topicGuidenceCmdPort


#################################################################################################################
#################################################################################################################
#################################################################################################################
class MAVSDK_Adapter():
    def __init__(self, log_dir, publishFrequency=100):
        self._log_dir = log_dir
        self._prev_alt_m = None
        self._alt_vel_count = 0
        self._prev_vel_vertical = 0
        self._prev_alt_ts = time.time()
        self._vertical_speed_filter = Low_Pass_Filter(alpha=0.1, is_angle=False)
        self._altitude_filter = Low_Pass_Filter(alpha=0.3, is_angle=False) # was 0.3
        
        self._publishDT = 1.0/publishFrequency
        self.flight_data = {}
        self._mavlink_parser = Mavlink_Parser()
        self._current_airspeed = 0

        self._offboard_control_enabled = False

#################################################################################################################
    async def run(self):
        # Start the tasks
        drone = System()
        lock = asyncio.Lock()
        await drone.connect(system_address='udp://:14540')
        tasks = [asyncio.ensure_future(self.attitude(drone, lock)),
                 asyncio.ensure_future(self.highresimu(drone, lock)),   #TODO: add highresimu to mavsdk
                 asyncio.ensure_future(self.attitudequat(drone, lock)),
                 asyncio.ensure_future(self.odometry(drone, lock)),
                 asyncio.ensure_future(self.positionned(drone, lock)),
                 asyncio.ensure_future(self.globalpositionint(drone, lock)),
                 asyncio.ensure_future(self.altitudem(drone, lock)),
                 asyncio.ensure_future(self.status_text(drone, lock)),
                 asyncio.ensure_future(self.velocity_ned(drone, lock)),
                 asyncio.ensure_future(self.scaled_imu(drone, lock)),
                 asyncio.ensure_future(self.scaled_pressure(drone, lock)),
                 asyncio.ensure_future(self.rc_status(drone, lock)),
                 asyncio.ensure_future(self.raw_imu(drone, lock)),
                 asyncio.ensure_future(self.listenerToCommands(drone)),
                 asyncio.ensure_future(self.publish_data(drone, lock)),
                 ]

        await asyncio.gather(*tasks)

    async def attitude(self, drone, lock):
        async for attitude in drone.telemetry.attitude_euler():
            async with lock:
                self.flight_data['local_ts'] = time.time()
                self.flight_data['attitude_timestamp'] = attitude.timestamp_us/1000.0
                self.flight_data['roll_deg'] = attitude.roll_deg
                self.flight_data['pitch_deg'] = attitude.pitch_deg
                self.flight_data['yaw_deg'] = attitude.yaw_deg

            # sockPub.send_multipart([zmqTopics.topicMavlinkAttitude, pickle.dumps(data)])
    
    async def attitudeRate(self, drone, lock):
        async for attitudeRate in drone.telemetry.attitude_angular_velocity():
            async with lock:
                self.flight_data['local_ts'] = time.time()
                self.flight_data['attitude_timestamp'] = attitudeRate.timestamp_us/1000.0
                self.flight_data['roll_deg_rate'] = attitudeRate.roll_deg
                self.flight_data['pitch_deg_rate'] = attitudeRate.pitch_deg
                self.flight_data['yaw_deg_rate'] = attitudeRate.yaw_deg
                self.flight_data['throttle'] = attitudeRate.thrust_percentage

            # sockPub.send_multipart([zmqTopics.topicMavlinkAttitude, pickle.dumps(data)])

    async def highresimu(self, drone, lock):
        async for imu in drone.telemetry.imu():
            async with lock:
                self.flight_data['local_ts'] = time.time()
                self.flight_data['imu_ts'] = imu.timestamp_us/1000.0
                self.flight_data['accl_frd_forward_m_s2'] = imu.acceleration_frd.forward_m_s2
                self.flight_data['accl_frd_right_m_s2'] = imu.acceleration_frd.right_m_s2
                self.flight_data['accl_frd_down_m_s2'] = imu.acceleration_frd.down_m_s2
                self.flight_data['gyro_frd_forward_rad_s'] = imu.angular_velocity_frd.forward_rad_s
                self.flight_data['gyro_frd_right_rad_s'] = imu.angular_velocity_frd.right_rad_s
                self.flight_data['gyro_frd_down_rad_s'] = imu.angular_velocity_frd.down_rad_s
                self.flight_data['mag_frd_forward_gauss'] = imu.magnetic_field_frd.forward_gauss
                self.flight_data['mag_frd_right_gauss'] = imu.magnetic_field_frd.right_gauss
                self.flight_data['mag_frd_down_gauss'] = imu.magnetic_field_frd.down_gauss
                self.flight_data['temperature_degc'] = imu.temperature_degc
            
            # sockPub.send_multipart([zmqTopics.topicMavlinkHighresIMU, pickle.dumps(data)])

    async def attitudequat(self, drone, lock):
        async for attitudequat in drone.telemetry.attitude_quaternion():
            async with lock:
                self.flight_data['local_ts'] = time.time()
                self.flight_data['quat_ts'] = attitudequat.timestamp_us/1000.0
                self.flight_data['quat_w'] = attitudequat.w
                self.flight_data['quat_x'] = attitudequat.x
                self.flight_data['quat_y'] = attitudequat.y
                self.flight_data['quat_z'] = attitudequat.z

            # sockPub.send_multipart([zmqTopics.topicMavlinkAttitudeQuat, pickle.dumps(data)])

    async def odometry(self, drone, lock):
        async for odometry in drone.telemetry.odometry():
            async with lock:
                self.flight_data['local_ts'] = time.time()
                self.flight_data['timestamp'] = odometry.time_usec/1000.0
                self.flight_data['angle_rate_body_roll_rad_s'] = odometry.angular_velocity_body.roll_rad_s
                self.flight_data['angle_rate_body_pitch_rad_s'] = odometry.angular_velocity_body.pitch_rad_s
                self.flight_data['angle_rate_body_yaw_rad_s'] = odometry.angular_velocity_body.yaw_rad_s           
                # data['pose_cov_mat'] = odometry.pose_covariance.covariance_matrix            
                self.flight_data['pos_body_x_m'] = odometry.position_body.x_m
                self.flight_data['pos_body_y_m'] = odometry.position_body.y_m
                self.flight_data['pos_body_z_m'] = odometry.position_body.z_m
                self.flight_data['quat_w'] = odometry.q.w
                self.flight_data['quat_x'] = odometry.q.x
                self.flight_data['quat_y'] = odometry.q.y
                self.flight_data['quat_z'] = odometry.q.z
                self.flight_data['quat_ts'] = self.flight_data['timestamp']

                self.flight_data['vel_body_x_m_s'] = odometry.velocity_body.x_m_s
                self.flight_data['vel_body_y_m_s'] = odometry.velocity_body.y_m_s
                self.flight_data['vel_body_z_m_s'] = odometry.velocity_body.z_m_s

                self.flight_data['quat_ts'] = self.flight_data['timestamp']
                # data['velocity_covariance'] = odometry.velocity_covariance.covariance_matrix
                self.flight_data['frame_id'] = odometry.frame_id.name
                self.flight_data['child_frame_id'] = odometry.child_frame_id.name
            
            # sockPub.send_multipart([zmqTopics.topicMavlinkOdometry, pickle.dumps(data)])

    async def positionned(self, drone, lock):
        async for positionned in drone.telemetry.position_velocity_ned():
            async with lock:
                self.flight_data['local_ts'] = time.time()
                self.flight_data['pos_north_m'] = positionned.position.north_m
                self.flight_data['pos_east_m'] = positionned.position.east_m
                self.flight_data['pos_down_m'] = positionned.position.down_m
                self.flight_data['vel_north_m_s'] = positionned.velocity.north_m_s
                self.flight_data['vel_east_m_s'] = positionned.velocity.east_m_s
                self.flight_data['vel_down_m_s'] = positionned.velocity.down_m_s
                # sockPub.send_multipart([zmqTopics.topicMavlinkLocalPositionNed, pickle.dumps(data)])

    async def globalpositionint(self, drone, lock):
        async for positionlla in drone.telemetry.position():
            async with lock:
                self.flight_data['local_ts'] = time.time()
                self.flight_data['global_lat_deg'] = positionlla.latitude_deg
                self.flight_data['global_lon_deg'] = positionlla.longitude_deg
                self.flight_data['global_alt_m'] = positionlla.absolute_altitude_m
                self.flight_data['global_relative_alt_m'] = positionlla.relative_altitude_m
            # sockPub.send_multipart([zmqTopics.topicMavlinkGlobalPositionInt, pickle.dumps(data)])

    async def altitudem(self, drone, lock):
        async for altitudem in drone.telemetry.altitude():
            async with lock:
                self.flight_data['local_ts'] = time.time()
                self.flight_data['altitude_m_amsl'] = altitudem.altitude_amsl_m
                self.flight_data['altitude_m_local'] = altitudem.altitude_local_m
                self.flight_data['altitude_m_monotonic'] = altitudem.altitude_monotonic_m
                self.flight_data['altitude_m_relative'] = altitudem.altitude_relative_m
                self.flight_data['altitude_m_terrain'] = altitudem.altitude_terrain_m
                self.flight_data['altitude_m_bottom_clearance'] = altitudem.bottom_clearance_m

            # sockPub.send_multipart([zmqTopics.topicMavlinkAltitude, pickle.dumps(data)])
                        
    async def status_text(self, drone, lock):
        async for status_text in drone.telemetry.status_text():
            async with lock:
                self.flight_data['local_ts'] = time.time()
                self.flight_data['status_text'] = status_text.text
     
    async def velocity_ned(self, drone, lock):
        async for velocity_ned in drone.telemetry.velocity_ned():
            async with lock:
                self.flight_data['local_ts'] = time.time()
                self.flight_data['vel_north_m_s'] = velocity_ned.north_m_s
                self.flight_data['vel_east_m_s'] = velocity_ned.east_m_s
                self.flight_data['vel_down_m_s'] = velocity_ned.down_m_s
                # sockPub.send_multipart([zmqTopics.topicMavlinkLocalPositionNed, pickle.dumps(data)])

    async def scaled_imu(self, drone, lock):
        async for scaled_imu in drone.telemetry.scaled_imu():
            async with lock:
                self.flight_data['local_ts'] = time.time()
                self.flight_data['imu_ts'] = scaled_imu.timestamp_us/1000.0
                self.flight_data['imu_ned_accel_x_m_s2'] = scaled_imu.acceleration_frd.forward_m_s2
                self.flight_data['imu_ned_accel_y_m_s2'] = scaled_imu.acceleration_frd.right_m_s2
                self.flight_data['imu_ned_accel_z_m_s2'] = scaled_imu.acceleration_frd.down_m_s2
            
    async def scaled_pressure(self, drone, lock):
        async for scaled_pressure in drone.telemetry.scaled_pressure():
            async with lock:
                self.flight_data['local_ts'] = time.time()
                self.flight_data['timestamp'] = scaled_pressure.timestamp_us/1000.0
                self.flight_data['absolute_press_hpa'] = scaled_pressure.absolute_pressure_hpa
                self.flight_data['differential_press_hpa'] = scaled_pressure.differential_pressure_hpa
                self.flight_data['temperature_degc'] = scaled_pressure.temperature_deg
                self.flight_data['differential_press_temperature_deg'] = scaled_pressure.differential_pressure_temperature_deg

    async def rc_status(self, drone, lock):
        async for rc_status in drone.telemetry.rc_status():
            async with lock:
                self.flight_data['local_ts'] = time.time()
                self.flight_data['is_available'] = rc_status.is_available
                self.flight_data['signal_strength_percent'] = rc_status.signal_strength_percent
                self.flight_data['was_available_once'] = rc_status.was_available_once

    async def raw_imu(self, drone, lock):
        async for raw_imu in drone.telemetry.raw_imu():
            async with lock:
                self.flight_data['local_ts'] = time.time()
                self.flight_data['imu_ts'] = raw_imu.timestamp_us/1000.0
                self.flight_data['imu_raw_frd_accel_x_m_s2'] = raw_imu.acceleration_frd.forward_m_s2
                self.flight_data['imu_raw_frd_accel_y_m_s2'] = raw_imu.acceleration_frd.right_m_s2
                self.flight_data['imu_raw_frd_accel_z_m_s2'] = raw_imu.acceleration_frd.down_m_s2
                self.flight_data['imu_raw_frd_gyro_x_rad_s'] = raw_imu.angular_velocity_frd.forward_rad_s
                self.flight_data['imu_raw_frd_gyro_y_rad_s'] = raw_imu.angular_velocity_frd.right_rad_s
                self.flight_data['imu_raw_frd_gyro_z_rad_s'] = raw_imu.angular_velocity_frd.down_rad_s
            
    async def raw_gps(self, drone, lock):
        async for raw_gps in drone.telemetry.raw_gps():
            async with lock:
                self.flight_data['local_ts'] = time.time()
                self.flight_data['timestamp'] = raw_gps.timestamp_us/1000.0
                self.flight_data['raw_lat_deg'] = raw_gps.latitude_deg
                self.flight_data['raw_lon_deg'] = raw_gps.longitude_deg
                self.flight_data['raw_alt_m'] = raw_gps.altitude_m
            
    async def heading(self, drone, lock):
        async for heading in drone.telemetry.heading():
            async with lock:
                self.flight_data['local_ts'] = time.time()
                self.flight_data['heading'] = heading.heading_deg
            
    async def health(self, drone, lock):
        async for health in drone.telemetry.health():
            async with lock:
                self.flight_data['local_ts'] = time.time()
                self.flight_data['is_gyrometer_calibration_ok'] = health.is_gyrometer_calibration_ok
                self.flight_data['is_accelerometer_calibration_ok'] = health.is_accelerometer_calibration_ok
                self.flight_data['is_magnetometer_calibration_ok'] = health.is_magnetometer_calibration_ok

    async def flight_mode(self, drone, lock):
        async for flight_mode in drone.telemetry.flight_mode():
            async with lock:
                self.flight_data['local_ts'] = time.time()
                self.flight_data['flight_mode'] = flight_mode.flight_mode
            
    async def ground_truth(self, drone, lock):
        async for ground_truth in drone.telemetry.ground_truth():
            async with lock:
                self.flight_data['local_ts'] = time.time()
                self.flight_data['raw_lat_deg'] = ground_truth.latitude_deg
                self.flight_data['raw_lon_deg'] = ground_truth.longitude_deg
                self.flight_data['raw_alt_m'] = ground_truth.altitude_m
    
    async def in_air(self, drone, lock):
        async for in_air in drone.telemetry.in_air():
            async with lock:
                self.flight_data['local_ts'] = time.time()
                self.flight_data['in_air'] = in_air.in_air
            
    async def landing_state(self, drone, lock):
        async for landing_state in drone.telemetry.landing_state():
            async with lock:
                self.flight_data['local_ts'] = time.time()
                self.flight_data['landing_state'] = landing_state.landing_state
    
    async def publish_data(self, drone, lock):
        while True:
            async with lock:
                sockPub.send_multipart([zmqTopics.topicMavlinkFlightData, pickle.dumps(self.flight_data)])
                if 'timestamp' in self.flight_data.keys():
                    print("self.flight_data['timestamp'] ", self.flight_data['timestamp'])
            await asyncio.sleep(self._publishDT)

################################################################################################################
    async def listenerToCommands(self, drone):
        while True:
            subSock = zmqWrapper.subscribe([zmqTopics.topicGuidenceCmdAttitude, 
                                zmqTopics.topicGuidenceCmdVelNedYaw,
                                zmqTopics.topicGuidenceCmdVelBodyYawRate,
                                zmqTopics.topicGuidenceCmdTakeoff,
                                zmqTopics.topicGuidenceCmdLand,
                                zmqTopics.topicGuidanceCmdArm,
                                ], zmqTopics.topicGuidenceCmdPort)
            ret = zmq.select([subSock], [], [], timeout=0.01)
            # ret = ret[0]
            if ret[0] is None or len(ret[0]) == 0:
                subSock.close()
                await asyncio.sleep(0.1)
                continue
            data = subSock.recv_multipart()
            subSock.close()
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
    asyncio.run(MAVSDK_Adapter("logs").run())


    


