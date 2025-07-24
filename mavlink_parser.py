#!/bin/python3
from common import Flight_Data, Quaternion, FLIGHT_MODE, LLA, Utils
from pymavlink import mavutil
import math
import numpy as np
import time
TIMESTAMP_FIELD_NAME = 'time_boot_ms'
INVALID_AUTOPILOT_ID = 8
MAX_ALT_JUMP = 10
MAX_LOCAL_TO_CORRECTED_TS_DIFF_SEC = 0.2
class Mavlink_Parser():
    def __init__(self):
        self._received_alt_data = False
        self._received_attitude_quat = False
        self._initial_pressure = None
        self._last_imu_msg_time = 0
        self._last_highres_imu_time = 0
        self._prev_alt = None
        self._last_timesync_ts = 0
        self._average_ts_diff_sec = None
        self._last_pressure_msg_time = 0
        self._last_attitude_quat_msg_time = 0
        self._last_attitude_msg_time = 0

    def _correct_local_ts(self, local_ts, time_boot_ms):
        return local_ts
        if(self._average_ts_diff_sec is None):
            return local_ts
        corrected_ts = (time_boot_ms/1000)+self._average_ts_diff_sec
        if(abs(corrected_ts - local_ts) > MAX_LOCAL_TO_CORRECTED_TS_DIFF_SEC):
            print("LARGE ERROR BETWEEN CORRECTED AND ORIGINAL TS:", abs(corrected_ts - local_ts))
            return local_ts
        return corrected_ts

    def get_current_offset(self):
        offset = self._average_ts_diff_sec
        if(offset is None):
            return 0
        return offset
    
    def parse(self, msg_dict, current_data:Flight_Data):
        success = False
        for key in msg_dict:
            msg = msg_dict[key]
            local_ts = float(msg['local-ts'])
            if(key == 'TIMESYNC'):
                # if(msg['tc1'] != 0):
                if(local_ts!=self._last_timesync_ts):
                    # print("timesync msg:",msg)
                    self._last_timesync_ts = local_ts
                    original_ts = float(msg['ts1'])/1000000000
                    mavlink_ts = float(msg['tc1'])/1000000000
                    # offset = tsync.ts1 + now_ns - tsync.tc1 * 2) / 2
                    diff = (original_ts + local_ts - (mavlink_ts*2))/2
                    diff_2 = original_ts-mavlink_ts
                    # print(local_ts,diff, diff_2)
                    # original_ts = diff + mavlink_ts
                    if(self._average_ts_diff_sec is None):
                        self._average_ts_diff_sec = diff
                    self._average_ts_diff_sec = Utils.rolling_avg(self._average_ts_diff_sec, diff, window_size=2)
                    # print("avg diff", self._average_ts_diff_sec)
            
            if(self._average_ts_diff_sec is None):
                time.sleep(0.001)
                continue
            if(key == 'VFR_HUD'):
                # print(msg)
                
                throttle = float(msg['throttle'])
                # if(throttle != 0):
                #     current_data.current_thrust = throttle/100
            elif(key == 'SCALED_PRESSURE'):
                if(local_ts != self._last_pressure_msg_time):
                    # print("pressure elapsed:",1/(local_ts -self._last_pressure_msg_time))
                    self._last_pressure_msg_time = local_ts

                # if(current_data.altitude_m.timestamp != local_ts):
                    corrected_ts = self._correct_local_ts(local_ts=local_ts, time_boot_ms=float(msg['time_boot_ms']))
                    # print("local_ts", local_ts, current_data.altitude_m.timestamp, local_ts-current_data.altitude_m.timestamp, current_data.altitude_m.timestamp != local_ts)

                    # local_ts = self._correct_local_ts(local_ts, msg['time_boot_ms'])
                    # print("correct ts",correct_ts, local_ts,correct_ts-local_ts)
                    current_pressure = float(msg['press_abs'])
                    current_temp = float(msg['temperature'])/100
                    if(self._initial_pressure is None):
                        self._initial_pressure = current_pressure
                    current_alt = Utils.get_height_diff_from_pressure_diff(pressure_start=self._initial_pressure, pressure_current=current_pressure, temperature_celcius=current_temp)
                    if(self._prev_alt is not None):
                        ####### THIS IS NEW-SHOULD BE CHECKED #######
                        if(abs(current_alt-self._prev_alt) > MAX_ALT_JUMP):
                            current_alt = self._prev_alt
                    self._prev_alt = current_alt
                    if(not self._received_alt_data):
                            current_data.altitude_m.relative = current_alt
                    current_data.altitude_m.amsl = current_alt
                    current_data.altitude_m.timestamp = corrected_ts

            elif(key == 'SCALED_IMU2'):
                if(local_ts != self._last_imu_msg_time):    
                    current_data.imu.timestamp = local_ts
                    corrected_ts = self._correct_local_ts(local_ts=local_ts, time_boot_ms=float(msg['time_boot_ms']))
                    local_ts = float(local_ts)
                    imu_freq = 1/(local_ts-self._last_imu_msg_time)
                    if(imu_freq < 20):
                        print("ERROR: IMU FREQUENCY is less than 20 HZ",imu_freq)
                    # print("imu_freq: ",1/(local_ts-self._last_imu_msg_time))
                    self._last_imu_msg_time = local_ts
                    # print("local_ts",local_ts)
                    xacc = float(msg['xacc'])/100
                    yacc = float(msg['yacc'])/100
                    zacc = float(msg['zacc'])/100
                    xgyro = float(msg['xgyro'])
                    ygyro = float(msg['ygyro'])
                    zgyro = float(msg['zgyro'])
                    current_data.imu.timestamp = corrected_ts
                    current_data.imu.gyro.set(xgyro, ygyro, zgyro)
                    # print("zacc:",zacc)
                    current_data.imu.accel.set(xacc, yacc, zacc)

            elif(key =='ATTITUDE_QUATERNION'):  # 50 HZ
                if(local_ts != self._last_attitude_quat_msg_time):
                    corrected_ts = self._correct_local_ts(local_ts=local_ts, time_boot_ms=float(msg['time_boot_ms']))
                    self._last_attitude_quat_msg_time = local_ts
                    w = float(msg['q1'])
                    x = float(msg['q2'])
                    y = float(msg['q3'])
                    z = float(msg['q4'])
                    current_data.orientation_rad.set(x=x, y=y,z=z,w=w,timestamp=local_ts) 
                    self._received_attitude_quat = True
                    # print("quat:",Quaternion.quat_to_euler(current_data.orientation_rad))
            elif(key == 'ATTITUDE'):
                if(not self._received_attitude_quat):
                    if(local_ts != self._last_attitude_msg_time):
                        self._last_attitude_msg_time = local_ts
                        corrected_ts = self._correct_local_ts(local_ts=local_ts, time_boot_ms=float(msg['time_boot_ms']))
                        roll = float(msg['roll'])
                        pitch = float(msg['pitch'])
                        yaw = float(msg['yaw'])
                        # print("orientation: roll: ",np.degrees(roll), "pitch: ", np.degrees(pitch), "yaw:", np.degrees(yaw))
                        # w = float(msg['q1'])
                        # x = float(msg['q2'])
                        # y = float(msg['q3'])
                        # z = float(msg['q4'])
                        current_data.orientation_rad = Quaternion.euler_to_quat(roll=roll, pitch=pitch, yaw=yaw)
                        current_data.orientation_rad.timestamp = local_ts
                        # current_data.orientation_rad.set(x=x, y=y,z=z,w=w,timestamp=local_ts) 
            #     print("attitude:",msg)
            elif(key == 'ATTITUDE_TARGET'):
                if(msg['thrust']==0):
                    current_data.current_thrust = 0.000001
                else:
                    current_data.current_thrust = msg['thrust']
            elif(key == 'GLOBAL_POSITION_INT'):
                
                controller_time_ms = int(msg['time_boot_ms'])
                current_data.pos_lla_deg.timestamp = float(local_ts)
                current_data.pos_lla_deg.lat = float(msg['lat'])/(10**7)
                current_data.pos_lla_deg.lon = float(msg['lon'])/(10**7)
                current_data.pos_lla_deg.alt = float(msg['alt'])/(10**3)
                current_data.pos_lla_deg.heading = float(msg['hdg'])/(100)
                # print("received LLA pos in mavlink", current_data.pos_lla_deg.lat, current_data.pos_lla_deg.lon)
            elif(key == 'ALTITUDE'): # 10 HZ
                if(local_ts > current_data.altitude_m.timestamp):
                    controller_time_microsec = int(msg['time_usec'])
                    dt =  local_ts - current_data.altitude_m.timestamp
                    # d_height = current_data.altitude_m.relative - float(msg['altitude_relative'])
                    # current_data.altitude_m.vertical_speed_estimate = d_height/dt
                    # current_data.altitude_m.timestamp = local_ts
                    if(not self._received_alt_data):
                        current_data.altitude_m.relative = float(msg['altitude_relative'])
                    current_data.altitude_m.amsl = float(msg['altitude_amsl'])
            elif(key == 'HIGHRES_IMU'): # 50 HZ
                if(local_ts != self._last_highres_imu_time): 
                    self._last_highres_imu_time = local_ts
                    print("CHECK IF THIS WORKS IN PX4")
                    corrected_ts = self._correct_local_ts(local_ts=local_ts, time_boot_ms=float(msg['time_boot_ms']))
                    controller_time_microsec = int(msg['time_usec'])
                    current_data.imu.timestamp = local_ts
                    accel_x = float(msg['xacc'])
                    accel_y = float(msg['yacc'])
                    accel_z = float(msg['zacc'])

                    gyro_x =  float(msg['xgyro'])
                    gyro_y =  float(msg['ygyro'])
                    gyro_z =  float(msg['zgyro'])
                    # current_data.altitude_m = float(msg['pressure_alt'])
                    dt =  local_ts - current_data.altitude_m.timestamp
                    if(current_data.altitude_m.relative != float(msg['pressure_alt'])):
                        d_height = current_data.altitude_m.relative - float(msg['pressure_alt'])
                        current_data.altitude_m.timestamp = local_ts

                        # current_data.altitude_m.vertical_speed_estimate = d_height/dt
                    # print("speed est",current_data.altitude_m.vertical_speed_estimate)
                    # current_data.altitude_m.timestamp = local_ts
                    current_data.altitude_m.relative = float(msg['pressure_alt'])
                    self._received_alt_data = True
                    # current_data.altitude_m.amsl = 0
                    current_data.imu.gyro.set(gyro_x, gyro_y, gyro_z)
                    

                    current_data.imu.accel.set(accel_x, accel_y, accel_z)
            elif(key == 'HEARTBEAT'):
                if(msg['autopilot'] == INVALID_AUTOPILOT_ID):
                    continue
                base_mode = msg['base_mode']
                armed = (base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) >> 7
                offboard = (base_mode & mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED) >> 3
                stabilized = (base_mode & mavutil.mavlink.MAV_MODE_FLAG_STABILIZE_ENABLED) >> 4 #MAV_MODE_FLAG_AUTO_ENABLED
                autonomous = (base_mode & mavutil.mavlink.MAV_MODE_FLAG_AUTO_ENABLED) >> 2

                offboard = (base_mode & mavutil.mavlink.MAV_MODE_FLAG_DECODE_POSITION_GUIDED)
                stabilized = (base_mode & mavutil.mavlink.MAV_MODE_FLAG_DECODE_POSITION_STABILIZE)
                autonomous = (base_mode & mavutil.mavlink.MAV_MODE_FLAG_DECODE_POSITION_AUTO)
                if(msg['mode_string'] == "UNKNOWN"):
                        msg['mode_string'] = 'OFFBOARD'
                # print("hb: ", msg, armed, offboard, stabilized, autonomous)
                # current_data.is_armed =  (bool(msg['base_mode']) & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                # print("hb:",msg)
                current_data.is_armed= bool(msg['base_mode'] & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                try:
                    
                    current_data.mode = FLIGHT_MODE[msg['mode_string']]
                except:
                    current_data.mode = FLIGHT_MODE.UNKNOWN
                
            elif(key== 'LOCAL_POSITION_NED' ):
                if(local_ts > current_data.pos_enu_m.timestamp):
                    # print(msg)
                    current_data.pos_enu_m.n = msg['x']
                    current_data.pos_enu_m.e = msg['y']
                    current_data.pos_enu_m.u = -msg['z']
                    current_data.pos_enu_m.vn = msg['vx']
                    current_data.pos_enu_m.ve = msg['vy']
                    current_data.pos_enu_m.vu = -msg['vz']
                    current_data.pos_enu_m.timestamp = local_ts
                    
        # return current_data

