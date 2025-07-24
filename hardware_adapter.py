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

import pymavlink

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
                                zmqTopics.topicGuidenceCmdVelYaw,
                                zmqTopics.topicGuidenceCmdAccYaw,
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

        success = self._init_mavlink()
        self._current_data = Flight_Data()
        self._init_success = True
        if(not success):
            self._init_success = False
            print("mavlink failed to initialize")
            return
        # self._thread_frequency_hz = 300
    
        # main_frame_proc = multiprocessing.Process(target=self._main, args={})
        # main_frame_proc.start()
        self._main()

#################################################################################################################
    def init_succeeded(self):
        return self._init_success

#################################################################################################################
    def _main(self):
        output_dict = {}

        mavlink_logger = Logger("mavlink", log_dir=self._log_dir, save_log_to_file=True, print_logs_to_console=False, datatype="TXT")
        while(1):
            try:
                msg = self.mavlink_connection.recv_match( blocking=True, timeout=1)
            except Exception as e:
                # print(e)
                continue
            if(msg is None):
                continue
            if(msg.get_type() == "BAD_DATA"):
                continue

            msg_dict = msg.to_dict()
            msg_dict['local-ts'] = time.time()
            # if 'HIGHRES_IMU' in msg_dict['mavpackettype']:
            #     pass
            if(msg_dict['mavpackettype'] == 'HEARTBEAT'):
                # print("received msg",msg)
                current_mode = mavutil.mode_string_v10(msg)
                msg_dict['mode_string'] = ( mavutil.mode_string_v10(msg))
          
            
            if msg_dict['mavpackettype'] in RELEVANT_MAVLINK_MESSAGES:
                ind = RELEVANT_MAVLINK_MESSAGES.index(msg_dict['mavpackettype'])
                sockPub.send_multipart([pubTopicsList[ind][0], pickle.dumps(msg_dict)])
            else:
                pass
            mavlink_logger.log(str(msg_dict))

            ret = zmq.select([subSock], [], [], 0.001)[0]
            if len(ret)>0:
                data = subSock.recv_multipart()
                topic = data[0]
                data = pickle.loads(data[1])
                
                if topic == zmqTopics.topicGuidenceCmdAttitude:                           # mavlink ATTITUDE
                    msg = pickle.loads(data[1])
                    targetQuat = Quaternion(x=msg['quatNedDesBodyFrdCmd'][1], y=msg['quatNedDesBodyFrdCmd'][2], z=msg['quatNedDesBodyFrdCmd'][3], w=msg['quatNedDesBodyFrdCmd'][0])
                    rpyRateCmd = Rate_Cmd(roll=msg['rpyRateCmd'][0], pitch=msg['rpyRateCmd'][1], yaw=msg['rpyRateCmd'][2])
                    thrustCmd = msg['thrustCmd']
                    self._send_goal_attitude(targetQuat, rpyRateCmd, thrustCmd)
                    
                elif topic == zmqTopics.topicGuidenceCmdVelYaw:
                    msg = pickle.loads(data[1])
                    yawCmd = msg['yawCmd']
                    velCmd = msg['velCmd']
                    self._send_setpoint(pos=None, vel=velCmd, acc=None, yaw=yawCmd, yaw_rate=None)
                    
                elif topic == zmqTopics.topicGuidenceCmdAccYaw:
                    msg = pickle.loads(data[1])
                    yawCmd = msg['yawCmd']
                    accCmd = msg['accCmd']
                    self._send_setpoint(pos=None, vel=None, acc=accCmd, yaw=yawCmd, yaw_rate=None)
                    
                elif topic == zmqTopics.topicGuidenceCmdAttitude:
                    msg = pickle.loads(data[1])
                    targetQuat = Quaternion(x=msg['quatNedDesBodyFrdCmd'][1], y=msg['quatNedDesBodyFrdCmd'][2], z=msg['quatNedDesBodyFrdCmd'][3], w=msg['quatNedDesBodyFrdCmd'][0])
                    rpyRateCmd = Rate_Cmd(roll=msg['rpyRateCmd'][0], pitch=msg['rpyRateCmd'][1], yaw=msg['rpyRateCmd'][2])
                    thrustCmd = msg['thrustCmd']
                    isRate = msg['isRate']
                    if isRate:
                        self._send_goal_attitude(goal_thrust=thrustCmd, goal_attitude=None, rates=rpyRateCmd)
                    else:
                        self._send_goal_attitude(goal_thrust=thrustCmd, goal_attitude=targetQuat, rates=None)
                    
                elif topic == zmqTopics.topicGuidenceCmdTakeoff:
                    msg = pickle.loads(data[1])
                    takeoff_altitude = msg['takeoff_altitude']
                    self._send_takeoff_cmd(takeoff_altitude)
                    
                elif topic == zmqTopics.topicGuidenceCmdLand:
                    self._send_land_cmd()
                    
                elif topic == zmqTopics.topicGuidanceCmdArm:
                    msg = pickle.loads(data[1])
                    self._arm()
                    
        
#################################################################################################################

    def _send_takeoff_cmd(self, takeoff_altitude = 10):
        if(self._mavlink_connected_to_usb):
            print("when connected to real flight controller takeoff is currently not allowed")
            return
        if(self._disable_offboard_control):
            return 
 
        print("Connected to PX4 autopilot")
        print(self.mavlink_connection.mode_mapping())
        mode_id = self.mavlink_connection.mode_mapping()["TAKEOFF"][1]
        print(mode_id)
        msg = self.mavlink_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        starting_alt = msg.alt / 1000
        takeoff_params = [0, 0, 0, 0, float("NAN"), float("NAN"), starting_alt + takeoff_altitude]
        time.sleep(1)
        # Change mode to takeoff (PX4)
        self.mavlink_connection.mav.command_long_send(self.mavlink_connection.target_system,
            self.mavlink_connection.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                    0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
        ack_msg = self.mavlink_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        print(f"Change Mode ACK:  {ack_msg}")
        time.sleep(1)
 
         # Command Takeoff
        self.mavlink_connection.mav.command_long_send(
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, takeoff_params[0], takeoff_params[1], takeoff_params[2], takeoff_params[3], takeoff_params[4], takeoff_params[5], takeoff_params[6])
        
        time.sleep(1)
       # Arm the UAS
        self.mavlink_connection.mav.command_long_send(self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
                                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        arm_msg = self.mavlink_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
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
        print(self.mavlink_connection.mode_mapping())
        mode_id = self.mavlink_connection.mode_mapping()["TAKEOFF"][1]
        print(mode_id)
        msg = self.mavlink_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        starting_alt = msg.alt / 1000
        takeoff_params = [0, 0, 0, 0, float("NAN"), float("NAN"), starting_alt + 1]
        time.sleep(1)
        # Change mode to takeoff (PX4)
        self.mavlink_connection.mav.command_long_send(self.mavlink_connection.target_system,
            self.mavlink_connection.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                    0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
        ack_msg = self.mavlink_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        print(f"Change Mode ACK:  {ack_msg}")
        time.sleep(1)
 
        #  Command Takeoff
        self.mavlink_connection.mav.command_long_send(
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, takeoff_params[0], takeoff_params[1], takeoff_params[2], takeoff_params[3], takeoff_params[4], takeoff_params[5], takeoff_params[6])
        
        time.sleep(1)
       # Arm the UAS
        self.mavlink_connection.mav.command_long_send(self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
                                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        arm_msg = self.mavlink_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
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
    def _init_mavlink(self):
        try:
            self.mavlink_connection = mavutil.mavlink_connection('udp:127.0.0.1:14540')
            self.mavlink_connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            # self.mavlink_connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            #                                 mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            ARDUPILOT = False
            if(ARDUPILOT):
                self.mavlink_connection.mav.request_data_stream_send(self.mavlink_connection.target_system, 
                                                                     self.mavlink_connection.target_component,
                                                                     mavutil.mavlink.MAV_DATA_STREAM_ALL,
                                                                     MAVLINK_RATE_HZ, 1)
        except Exception as e:
            print("failed to initialize mavlink connection with error: "+str(e))
            return
        count = 0
        success = True
        out = None
        while(count < 3):
            count = count + 1
            try:
                self.mavlink_connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                out = self.mavlink_connection.wait_heartbeat(timeout=2)
                if(out is not None):
                    break

            except:
                pass
        if(out is None):
            success = False
        else:
            self._mavlink_parser = Mavlink_Parser()
        #print("mode mapping:",self.mavlink_connection.mode_mapping())
        # self.arm()
        return success

#################################################################################################################
    def send_setpoint(self, pos=None, vel=None, acc=None, yaw=None, yaw_rate=None):
        if(self._disable_offboard_control):
            return
        time_boot_ms = 0
        coordinate_frame =mavutil.mavlink.MAV_FRAME_GLOBAL_INT
        target_system = self.mavlink_connection.target_system
        target_component = self.mavlink_connection.target_component
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
            type_mask = type_mask | mavutil.mavlink.MAV_POSITION_TARGET_TYPEMASK_X_IGNORE
            type_mask = type_mask | mavutil.mavlink.MAV_POSITION_TARGET_TYPEMASK_Y_IGNORE
            type_mask = type_mask | mavutil.mavlink.MAV_POSITION_TARGET_TYPEMASK_Z_IGNORE
            pos = np.zeros(3)
        if vel is None:
            type_mask = type_mask | mavutil.mavlink.MAV_POSITION_TARGET_TYPEMASK_VX_IGNORE
            type_mask = type_mask | mavutil.mavlink.MAV_POSITION_TARGET_TYPEMASK_VY_IGNORE
            type_mask = type_mask | mavutil.mavlink.MAV_POSITION_TARGET_TYPEMASK_VZ_IGNORE
            vel = np.zeros(3)
        if acc is None:
            type_mask = type_mask | mavutil.mavlink.MAV_POSITION_TARGET_TYPEMASK_AFX_IGNORE
            type_mask = type_mask | mavutil.mavlink.MAV_POSITION_TARGET_TYPEMASK_AFY_IGNORE
            type_mask = type_mask | mavutil.mavlink.MAV_POSITION_TARGET_TYPEMASK_AFZ_IGNORE
            acc = np.zeros(3)
        if yaw is None:
            type_mask = type_mask | mavutil.mavlink.MAV_POSITION_TARGET_TYPEMASK_YAW_IGNORE
            yaw = 0
        if yaw_rate is None:
            type_mask = type_mask | mavutil.mavlink.MAV_POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
            yaw_rate = 0
        north = pos[0];        east = pos[1];        down = pos[2]
        vx = vel[0];        vy = vel[1];        vz = vel[2]
        afx = acc[0];        afy = acc[1];        afz = acc[2]

        self.mavlink_connection.mav.set_position_target_local_ned_send(
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

        if(self._disable_offboard_control):
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
        self.mavlink_connection.mav.set_attitude_target_send(
            time_boot_ms,
            self.mavlink_connection.target_system,
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
        if mode not in  self.mavlink_connection.mode_mapping():
            print("unknown mode")
        else:
            
            mode_id = self.mavlink_connection.mode_mapping()[mode]
            self.mavlink_connection.mav.command_long_send(
                self.mavlink_connection.target_system, 
                self.mavlink_connection.target_component,
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
        self.mavlink_connection.mav.command_long_send(
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0)

#################################################################################################################
    def _send_land_cmd(self):
        self.mavlink_connection.mav.command_long_send(
            self.mavlink_connection.target_system,
            self.mavlink_connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)

#################################################################################################################
if __name__ == '__main__':
    hardware_adapter = Hardware_Adapter(log_dir="logs")
    # hw_adapter = Hardware_Adapter(config_dir="config", use_usb=True)
    # if(not hw_adapter.init_succeeded()):
    #     raise Exception ("failed to init")
    # is_armed = False
    # is_offboard = False
    # count = 0
    # # hw_adapter.switch_to_offboard()
    # start = time.time()
    # while(1): 
    #     count = count +1
    #     current_data = hw_adapter.step()
    #     if(count % 10 == 0 and 1):
    #         print("current_data",current_data)
    #     if(time.time()-start > 0):
    #         hw_adapter.send_pos_cmd_DEBUG(lat=47.3974805, lon=8.5456492, alt=498.187)
    #         # hw_adapter.send_attitude_cmd(Quaternion.euler_to_quat(roll=0, pitch=np.radians(0), yaw=0),0.6)
    #     time.sleep(0.02)