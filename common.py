#!/bin/bash/python3
import pickle
import os
import numpy as np
from numpy import sin, cos, tan, arctan, arctan2, arcsin, arccos, pi
from multiprocessing import  Lock
from matplotlib import pyplot as plt
import time
import math
import numpy as np
from datetime import datetime
from datetime import timezone
import subprocess
from copy import deepcopy
ROUND_VAL = 4
QUEUE_READ_TIMEOUT = 0.1
USB_DISK_NAME_LIST =["SanDisk"]
from enum import Enum

#################################################################################################################
#################################################################################################################
#################################################################################################################
class YAW_COMMAND(Enum):
    NO_CONTROL   = 0     # no yaw control
    DEFINED_DIR  = 1     # yaw control corresponding to defined direction
    HOLD_CUR_DIR = 2     # hold current direction
    VELOCITY_DIR = 3     # direction of drone corresponding to velocity direction
    CAMERA_DIR   = 4     # direction of drone corresponding to camera direction
    
#################################################################################################################
#################################################################################################################
#################################################################################################################
class VEHICLE_TYPE(Enum):
    GAZEBO = 0
    AIRSIM = 1
    FOXTECH = 2

#################################################################################################################
#################################################################################################################
#################################################################################################################
class TUNING_SUBSTATE(Enum):
    NONE = 0
    VELOCITY = 1
    ACCELERATION = 2
    PITCH = 3
    ROLL = 4
    ANGLE = 5
    POSITION = 6
    THRUST = 7
    PITCH_AND_THRUST = 8
    VERTICAL = 9
    HORIZONTAL = 10
    @staticmethod
#################################################################################################################
    def from_str(label:str):
        label = label.upper()
        if(label == "NONE"):
            return TUNING_SUBSTATE.NONE
        elif(label == "VELOCITY" or label == "VEL"):
            return TUNING_SUBSTATE.VELOCITY
        elif(label == "ACCELERATION" or label == "ACCEL"):
            return TUNING_SUBSTATE.ACCELERATION
        elif(label == "PITCH"):
            return TUNING_SUBSTATE.PITCH
        elif(label == "ROLL"):
            return TUNING_SUBSTATE.ROLL
        elif(label == "ANGLE"):
            return TUNING_SUBSTATE.ANGLE
        elif(label == "VELOCITY"):
            return TUNING_SUBSTATE.VELOCITY
        elif(label == "POSITION" or label == "POS"):
            return TUNING_SUBSTATE.POSITION
        elif(label == "HORIZONTAL" or label == "HORIZ"):
            return TUNING_SUBSTATE.HORIZONTAL
        elif(label == "VERTICAL" or label ==  "VERT"):
            return TUNING_SUBSTATE.VERTICAL
        else:
            print("string",label, "not recognized as TUNING_SUBSTATE")
            return TUNING_SUBSTATE.NONE

#################################################################################################################
#################################################################################################################
#################################################################################################################
class TUNING_STATE(Enum):
    NONE = 0
    HOVER = 1
    PITCH = 2
    ROLL = 3
    YAW = 4
    THRUST = 5
    GUIDANCE = 6
    @staticmethod
#################################################################################################################
    def from_str(label:str):
        label = label.upper()
        if(label == "NONE"):
            return TUNING_STATE.NONE
        elif(label == "HOVER"):
            return TUNING_STATE.HOVER
        elif(label == "PITCH"):
            return TUNING_STATE.PITCH
        elif(label == "ROLL"):
            return TUNING_STATE.ROLL
        elif(label == "YAW"):
            return TUNING_STATE.YAW
        elif(label == "THRUST"):
            return TUNING_STATE.THRUST
        elif(label == "GUIDANCE"):
            return TUNING_STATE.GUIDANCE
        else:
            print("string",label, "not recognized as TUNING_STATE")
            return TUNING_STATE.NONE

#################################################################################################################
#################################################################################################################
#################################################################################################################
class Tuning():
    def __init__(self, state:TUNING_STATE, substate:TUNING_SUBSTATE, value, secondary_value=None):
        self.state = state
        self.substate = substate
        self.value = value
        self.secondary_value = secondary_value
#################################################################################################################
    def __str__(self):
        return "Tuning state: "+str(self.state)+" substate:"+str(self.substate) + " value: "+str(self.value)

#################################################################################################################
#################################################################################################################
#################################################################################################################
class CAMERA_TYPE(Enum):
    GAZEBO = 0
    AIRSIM = 1
    UNKNOWN = 2
    USB_H264 = 3
    REPLAY = 4
    @staticmethod
    def from_str(label:str):
        label = label.lower()
        if label in ('gazebo'):
            return CAMERA_TYPE.GAZEBO
        elif label in ('airsim'):
            return CAMERA_TYPE.AIRSIM
        elif label in ('usb_h264'):
            return CAMERA_TYPE.USB_H264
        elif label in ('replay'):
            return CAMERA_TYPE.REPLAY
        else:
            raise CAMERA_TYPE.UNKNOWN

#################################################################################################################
#################################################################################################################
#################################################################################################################
class LPF_TYPE(Enum):
    FIRST_ORDER =0
    OTHER = 1

#################################################################################################################
#################################################################################################################
#################################################################################################################
class MAV_CMD(Enum):
    ARM = 0
    SET_TO_OFFBOARD = 1
    SET_ATTITUDE_TARGET = 2
    POS_SETPOINT_DEBUG = 3
    TAKEOFF = 4

#################################################################################################################
#################################################################################################################
#################################################################################################################
class FLIGHT_MODE(Enum):
    UNKNOWN=0
    OFFBOARD=1
    STABILIZED = 2
    MANUAL = 3
    ACRO = 4
    RATTITUDE = 5
    ALTCTL = 6
    POSCTL = 7
    LOITER = 8
    MISSION = 9
    RTL = 10
    LAND = 11
    RTGS = 12
    TAKEOFF = 13
    FOLLOWME = 14

#################################################################################################################
#################################################################################################################
#################################################################################################################
class LLA():  # lat, lon, alt
    def __init__(self, timestamp, lla = np.zeros(3), lla_vel = None, relative_alt = None):
        self.timestamp = timestamp
        self.lla = deepcopy(lla)
        self.lla_vel = lla_vel
        self.relative_alt = relative_alt
#################################################################################################################
    def __str__(self):
        return str("lat:"+str(self.lla[0])+" lon:"+str(self.lla[1])+" alt:"+str(self.lla[2]))

#################################################################################################################
#################################################################################################################
#################################################################################################################
class Virtual_Target():
    def __init__(self):
        pass

#################################################################################################################
#################################################################################################################
#################################################################################################################
def rotX(angle):
    return np.array([[1, 0, 0], [0, np.cos(angle), -np.sin(angle)], [0, np.sin(angle), np.cos(angle)]])

def rotY(angle):
    return np.array([[np.cos(angle), 0, np.sin(angle)], [0, 1, 0], [-np.sin(angle), 0, np.cos(angle)]])

def rotZ(angle):
    return np.array([[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
# #################################################################################################################
# #################################################################################################################
# #################################################################################################################
# class Anglular_Velocity():
#     def __init__(self, x, y, z):
#         self.x = x
#         self.y = y
#         self.z = z 

# #################################################################################################################
#     def set(self, x,y,z):
#         self.x = x
#         self.y = y
#         self.z = z 

#################################################################################################################
#################################################################################################################
#################################################################################################################
class Altitude():
    def __init__(self, altitude_amsl, altitude_relative, timestamp=0):
        self.timestamp = timestamp
        self.amsl = altitude_amsl
        self.relative = altitude_relative
        self.vertical_speed_estimate = 0

#################################################################################################################
#################################################################################################################
#################################################################################################################
class Imu():
    def __init__(self, timestamp, accel = np.zeros(3), gyro = np.zeros(3)):
        self.timestamp = timestamp
        self.accel = deepcopy(accel)
        self.gyro = deepcopy(gyro)
        
# #################################################################################################################
# #################################################################################################################
# #################################################################################################################
# class Acceleration():
#     def __init__(self, x, y, z):
#         self.x = x
#         self.y = y
#         self.z = z

#     def set(self, x,y,z):
#         self.x = x
#         self.y = y
#         self.z = z

#################################################################################################################
#################################################################################################################
#################################################################################################################
class Flight_Data():
    def __init__(self):
        self.quat_ts = 0
        self.quat_ned_bodyfrd = Quaternion(0,0,0,1)
        self.altitude_m = Altitude(0,0)
        self.is_armed = False
        self.mode = FLIGHT_MODE.UNKNOWN    
        self.imu_ts = 0                          # imu timestamp in milliseconds
        self.imu_raw_frd = Imu(0,np.zeros(3),np.zeros(3))  # imu - acceleration and gyro in m/s^2 and rad/s
        self.imu_ned = Imu(0,np.zeros(3),np.zeros(3))  # imu - acceleration and gyro in m/s^2 and rad/s
        self.pos_ned_m = NED()            # ned - east north up
        self.raw_pos_lla_deg = LLA(0,np.zeros(3))   # lla - lat lon altitude in degrees and meters
        self.filt_pos_lla_deg = LLA(0,np.zeros(3))   # lla - lat lon altitude in degrees and meters
        self.rpy_rates = np.zeros(3)                # roll, pitch, yaw rates in radians/s
        self.current_thrust = 0                      # current thrust in percentage
        self.rpy = np.zeros(3)           # roll, pitch, yaw in radians
        self.rpy2_rates = np.zeros(3)                # roll, pitch, yaw rates in radians/s
        self.custom_mode_id = None                  # custom mode id
        self.custom_mode_name = None                # custom mode name
        self.throttle = 0                           # throttle in percentage
        self.heading = 0                             # angle in from north in radians
        self.groundspeed = 0                         # ground speed in m/s
        self.offboardMode = False                    # offboard mode
        self.timestamp = 0               # timestamp in milliseconds
        self.local_ts = 0                # local timestamp in milliseconds
        self.temperature = 0             # temperature in degC
        self.amsl_m = 0                  # altitude above mean sea level in meters
        self.local_m = 0                 # altitude above local level in meters
        self.monotonic_m = 0             # altitude above takeoff in meters
        self.relative_m = 0              # altitude above home in meters
        self.terrain_m = 0               # altitude above terrain in meters
        self.bottom_clearance_m = 0      # clearance below the vehicle in meters
        self.status_text = None          # status text
        self.pressure = 0                # pressure in hPa
        self.temperature = 0             # temperature in degC
        self.absolute_press_hpa = 0      # absolute pressure in hPa
        self.differential_press_hpa = 0  # differential pressure in hPa
        self.is_available = False       # rc status is available
        self.signal_strength_percent = 0 # rc status signal strength in percentage
        self.was_available_once = False  # rc status was available once
        self.is_gyrometer_calibration_ok = False # gyrometer calibration is ok
        self.is_accelerometer_calibration_ok = False # accelerometer calibration is ok
        self.is_magnetometer_calibration_ok = False # magnetometer calibration is ok
        self.flight_mode = None # flight mode
        self.in_air = False # in air
        self.landing_state = None # landing state
        self.gathered={
            'euler_ned_bodyfrd':False,
            'quat_ned_bodyfrd':False,
            'pos_ned_m':False,
            'imu_ned':False,
            'tracker_px':False,
        }
#################################################################################################################
    def __str__(self):
        return "Flight Data: state:"+str(self.mode.name)+" armed:"+str(self.is_armed) +" pos: e:"+str(self.pos_ned_m.e) +" n: "+str(self.pos_ned_m.n)+" u: "+str(self.pos_ned_m.u)

#################################################################################################################
    def set_orientation(self, x,y,z,w):
        pass
#################################################################################################################
    def set_accelerometer(self, x,y,z):
        pass



#################################################################################################################
#################################################################################################################
#################################################################################################################
class APPROACH_PATH():
    HIGH = 7/8
    MEDIUM = 11/12
    STRAIGHT = 1
    LOW = 10/8

#################################################################################################################
#################################################################################################################
#################################################################################################################
class NED():   # north east down
    def __init__(self, ned=np.zeros(3), vel_ned=np.zeros(3), timestamp = 0):
        self.timestamp = timestamp
        self.ned = deepcopy(ned)
        self.vel_ned = deepcopy(vel_ned)

#################################################################################################################
    def clear(self):
        self.timestamp = 0
        self.ned = np.zeros(3)
        # self.vel_ned = np.zeros(3)

#################################################################################################################
    def __str__(self):
        return "e: "+str(round(self.ned[0], ROUND_VAL)) +" n: "+str(round(self.ned[1],ROUND_VAL))+" u: "+str(round(self.ned[2], ROUND_VAL))

#################################################################################################################
#################################################################################################################
#################################################################################################################
class Euler():
    def __init__(self, rpy):  # roll pitch yaw
        self.rpy = deepcopy(np.array(rpy))

#################################################################################################################
    def set(self, rpy):
        self.rpy = deepcopy(rpy)

#################################################################################################################
    @staticmethod
    def diff(e1, e2):
        return Euler(e1.rpy - e2.rpy)

#################################################################################################################
    def __str__(self):
        return str("roll: "+str(np.degrees(self.rpy[0]))+" pitch: "+str(np.degrees(self.rpy[1]))+" yaw: "+str(np.degrees(self.rpy[2])))

#################################################################################################################
    def to_quat(self):
        q = Quaternion(0,0,0,0)
        if(self.rpy is not None):
            cy = math.cos(self.rpy[2] * 0.5)
            sy = math.sin(self.rpy[2] * 0.5)
            cr = math.cos(self.rpy[0] * 0.5)
            sr = math.sin(self.rpy[0] * 0.5)
            cp = math.cos(self.rpy[1] * 0.5)
            sp = math.sin(self.rpy[1] * 0.5)
            q.w = cr * cp * cy + sr * sp * sy#
            q.x = sr * cp * cy - cr * sp * sy#
            q.y = cr * sp * cy + sr * cp * sy#
            q.z = cr * cp * sy - sr * sp * cy#
            q = Quaternion.normalize(q)
        return q
#################################################################################################################
#################################################################################################################
#################################################################################################################
class Rate_Cmd():
    def __init__(self, rpydot):
        self.rpydot = deepcopy(rpydot)

#################################################################################################################
def rpyRate2omega_frd(rpyVec, rpydot):
    #conversion from rpy rates to angular velocity
    convMat_ned_omega_rpy = np.array([[1,                    0,         -np.sin(rpyVec[1])                     ],
                                      [0,            np.cos(rpyVec[0]), np.cos(rpyVec[1]) * np.sin(rpyVec[0])],
                                      [0,           -np.sin(rpyVec[0]), np.cos(rpyVec[1]) * np.cos(rpyVec[0])]])
    
    omega_frd = convMat_ned_omega_rpy @ rpydot
    return omega_frd

def omega_frd2rpyRate(rpyVec, omega_frd):
    #conversion from angular velocity to rpy rates
    convMat_ned_rpy_omega = np.array([[1,            sin(rpyVec[0])*tan(rpyVec[1]), cos(rpyVec[0])*tan(rpyVec[1])],
                                      [0,            cos(rpyVec[0]), -sin(rpyVec[0])],
                                      [ 0,           sin(rpyVec[0])/cos(rpyVec[1]), cos(rpyVec[0])/cos(rpyVec[1])]])

    rpydot = convMat_ned_rpy_omega @ omega_frd
    return rpydot

# def EulerRates2BodyRates(pqr, dEdt):
#     """
#     Convert Euler-rates, dEdt, (time derivative of roll, pitch, yaw) into 
#     body rates.

#     Parameters:
#     pqr     : numpy array, roll-pitch-yaw zyx-Euler angles [roll, pitch, yaw]
#     dEdt    : numpy array, time derivative of the zyx-Euler angles [d_roll/dt, d_pitch/dt, d_yaw/dt]

#     Returns:
#     omega   : numpy array, body-rate angular velocity [omega_x, omega_y, omega_z]
#     """

#     # Extract roll, pitch, yaw from pqr
#     roll, pitch, yaw = pqr

#     # Compute sine and cosine of roll and pitch
#     sp = np.sin(roll)  # sin(phi)
#     cp = np.cos(roll)  # cos(phi)
#     sq = np.sin(pitch) # sin(theta)
#     cq = np.cos(pitch) # cos(theta)

#     # Transformation matrix T
#     T = np.array([
#         [1,   0,   -sq],
#         [0,  cp, cq*sp],
#         [0, -sp, cq*cp]
#     ])

#     # Compute body rates
#     omega = T @ dEdt

#     return omega
#################################################################################################################
#################################################################################################################
#################################################################################################################
class Quaternion():
    def __init__(self, x = 0, y = 0, z = 0, w = 1, timestamp=0):
        #TODO: add check for
        self.timestamp = timestamp
        self.x = x
        self.y = y
        self.z = z
        self.w = w
#################################################################################################################
    def __neg__(self):
            return Quaternion(x=-self.x, y=-self.y, z=-self.z, w=-self.w)  
################################################################################################################
    @staticmethod
    def from_matrix(mat):
        trace = mat[0][0] + mat[1][1] + mat[2][2]  # Same as mat[0][0] + mat[1][1] + mat[2][2]
        
        if trace > 0:
            s = 0.5 / math.sqrt(trace + 1.0)
            w = 0.25 / s  # w
            x = (mat[2][1] - mat[1][2]) * s  # x
            y = (mat[0][2] - mat[2][0]) * s  # y
            z = (mat[1][0] - mat[0][1]) * s  # z
        else:
            if mat[0][0] > mat[1][1] and mat[0][0] > mat[2][2]:
                s = 2.0 * math.sqrt(1.0 + mat[0][0] - mat[1][1] - mat[2][2])
                w = (mat[2][1] - mat[1][2]) / s  # w
                x = 0.25 * s  # x
                y = (mat[0][1] + mat[1][0]) / s  # y
                z = (mat[0][2] + mat[2][0]) / s  # z
            elif mat[1][1] > mat[2][2]:
                s = 2.0 * math.sqrt(1.0 + mat[1][1] - mat[0][0] - mat[2][2])
                w = (mat[0][2] - mat[2][0]) / s  # w
                x = (mat[0][1] + mat[1][0]) / s  # x
                y = 0.25 * s  # y
                z = (mat[1][2] + mat[2][1]) / s  # z
            else:
                s = 2.0 * math.sqrt(1.0 + mat[2][2] - mat[0][0] - mat[1][1])
                w = (mat[1][0] - mat[0][1]) / s  # w
                x = (mat[0][2] + mat[2][0]) / s  # x
                y = (mat[1][2] + mat[2][1]) / s  # y
                z = 0.25 * s  # z

        return Quaternion(x=x, y=y, z=z, w=w)
#################################################################################################################
    @staticmethod
    def from_axis_angle(axis:np.array, angle): 
        if np.linalg.norm(axis) == 0:
            return Quaternion(x = 0, y = 0, z = 0, w = 1)
        
        axis_ = axis/np.linalg.norm(axis)
        q = Quaternion(0,0,0,0)
        q.w = math.cos(angle/2)
        q.x = axis_[0] * math.sin(angle/2)
        q.y = axis_[1] * math.sin(angle/2)
        q.z = axis_[2] * math.sin(angle/2)
        return q
#################################################################################################################
    def to_rotation_matrix(self):
        """
        Covert a quaternion into a full three-dimensional rotation matrix.
    
        Input
        :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
    
        Output
        :return: A 3x3 element matrix representing the full 3D rotation matrix. 
                This rotation matrix converts a point in the local reference 
                frame to a point in the global reference frame.
        """
        # Extract the values from Q
        q0 = self.w
        q1 = self.x
        q2 = self.y
        q3 = self.z
        
        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)
        
        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)
        
        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1
        
        # 3x3 rotation matrix
        rot_matrix = np.array([[r00, r01, r02],
                            [r10, r11, r12],
                            [r20, r21, r22]])
                                
        return rot_matrix
        
    
    
    
    def set(self, x, y, z, w, timestamp=0):
        self.timestamp = timestamp
        self.x = x
        self.y = y
        self.z = z
        self.w = w

#################################################################################################################
    def __mul__(self, other):
        w = self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z
        x = self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y
        y = self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x
        z = self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w
        return Quaternion(x=x, y=y, z=z, w=w)

#################################################################################################################
    def __truediv__(self, scalar:float):
        return Quaternion(x=self.x/scalar, y=self.y/scalar, z=self.z/scalar, w=self.w/scalar)

#################################################################################################################
    def conjugate(self):
        return Quaternion(x=-self.x, y=-self.y, z=-self.z, w=self.w)

#################################################################################################################
    def inv(self):
        return Quaternion(x=-self.x, y=-self.y, z=-self.z, w=self.w)

#################################################################################################################
    def __str__(self):
        return str("x: "+str(self.x)+" y: "+str(self.y)+" z: "+str(self.z)+" w: "+str(self.w)+" ts: "+str(self.timestamp))
        
#################################################################################################################
    def reverse_rotate_vector(self, x,y,z):
        return self.conjugate().rotate_vector(x,y,z)

#################################################################################################################
    def rotate_vec(self, vec:np.array):
        res = (self*Quaternion(x=vec[0], y=vec[1], z=vec[2], w=0))*self.conjugate()
        return np.array([res.x, res.y, res.z])

#################################################################################################################
    def length(self, v):
        if(len(v)!=3):
            return np.nan
        return np.sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2])
#################################################################################################################
    def passive_rotate_vector(self, x,y,z):
        v_quat = Quaternion(x=x, y=y,z=z,w=0)
        q_conjugate = self.conjugate()
        rotated_v = self * v_quat * q_conjugate
        return ([rotated_v.x, rotated_v.y, rotated_v.z])

#################################################################################################################
    def active_rotate_vector(self, x,y,z):
        v_quat = Quaternion(x=x, y=y,z=z,w=0) 
        q_conjugate = self.conjugate()
        rotated_v = q_conjugate * v_quat * self
        return ([rotated_v.x, rotated_v.y, rotated_v.z])
#################################################################################################################
    def dot(self, quat):
        return self.x*quat.x + self.y*quat.y + self.z*quat.z + self.w*quat.w
#################################################################################################################
    @staticmethod
    def quat_dot(quat1, quat2):
        return quat1.x*quat2.x + quat1.y*quat2.y + quat1.z*quat2.z + quat1.w*quat2.w
#################################################################################################################
    @staticmethod
    def normalize(quat):
        magnitude = Quaternion.magnitude(quat)
        return Quaternion(x=quat.x/magnitude, y=quat.y/magnitude, z=quat.z/magnitude, w=quat.w/magnitude)
#################################################################################################################
    @staticmethod
    def quad_diff(quat_original, quat_base):
        conj = Quaternion.quat_conj(quat_original)
        mult = Quaternion.quat_mult(quat_base, conj)
        return mult        
#################################################################################################################
    @staticmethod
    def quat_mult(quat_1, quat_2):
        t0 = (quat_1.w*quat_2.w - quat_1.x*quat_2.x - quat_1.y*quat_2.y - quat_1.z*quat_2.z)
        t1 = (quat_1.w*quat_2.x + quat_1.x*quat_2.w + quat_1.y*quat_2.z - quat_1.z*quat_2.y)
        t2 = (quat_1.w*quat_2.y - quat_1.x*quat_2.z + quat_1.y*quat_2.w + quat_1.z*quat_2.x)
        t3 = (quat_1.w*quat_2.z + quat_1.x*quat_2.y - quat_1.y*quat_2.x + quat_1.z*quat_2.w)
        return Quaternion(t1, t2, t3, t0)

#################################################################################################################
    @staticmethod
    def quat_conj(quat):
        return Quaternion(-quat.x, -quat.y, -quat.z, quat.w)

#################################################################################################################
    @staticmethod
    def magnitude(quat):
        return math.sqrt(quat.x*quat.x + quat.y*quat.y+ quat.z*quat.z+ quat.w*quat.w)
        
#################################################################################################################
    @staticmethod
    def inverse(quat):
        magnitude = Quaternion.magnitude(quat)
        conj = Quaternion.quat_conj(quat)
        return Quaternion(x=conj.x/magnitude, y=conj.y/magnitude, z=conj.z/magnitude, w=conj.w/magnitude)

#################################################################################################################
    def to_euler(self):
        t0 = +2.0 * (self.w * self.x + self.y * self.z)
        t1 = +1.0 - 2.0 * (self.x * self.x + self.y * self.y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (self.w * self.y - self.z * self.x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (self.w * self.z + self.x * self.y)
        t4 = +1.0 - 2.0 * (self.y * self.y + self.z * self.z)
        yaw_z = math.atan2(t3, t4)
        return Euler(np.array([roll_x, pitch_y, yaw_z]))
    

    #     // roll (x-axis rotation)
    # double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    # double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    # angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    # // pitch (y-axis rotation)
    # double sinp = std::sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
    # double cosp = std::sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
    # angles.pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    # // yaw (z-axis rotation)
    # double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    # double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    # angles.yaw = std::atan2(siny_cosp, cosy_cosp);


#################################################################################################################
    @staticmethod
    def quat_to_euler(q):
        # roll (x-axis rotation)
        sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
        roll_x = np.arctan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = np.sqrt(1 + 2 * (q.w * q.y - q.x * q.z))
        cosp = np.sqrt(1 - 2 * (q.w * q.y - q.x * q.z))
        pitch_y = 2 * np.arctan2(sinp, cosp) - np.pi / 2

        # yaw (z-axis rotation)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw_z = np.arctan2(siny_cosp, cosy_cosp)

        return Euler(np.array([roll_x, pitch_y, yaw_z]))
#################################################################################################################
    @staticmethod
    def from_euler(roll, pitch, yaw):
        return Quaternion.euler_to_quat(roll=roll, pitch=pitch, yaw=yaw)
#################################################################################################################
    @staticmethod
    def euler_to_quat(euler=None, roll=None, pitch=None, yaw=None):
        if(euler is not None):
            q = Quaternion(0,0,0,0)
            cy = math.cos(euler.rpy[2] * 0.5)
            sy = math.sin(euler.rpy[2] * 0.5)
            cr = math.cos(euler.rpy[0] * 0.5)
            sr = math.sin(euler.rpy[0] * 0.5)
            cp = math.cos(euler.rpy[1] * 0.5)
            sp = math.sin(euler.rpy[1] * 0.5)
            q.w = cr * cp * cy + sr * sp * sy#
            q.x = sr * cp * cy - cr * sp * sy#
            q.y = cr * sp * cy + sr * cp * sy#
            q.z = cr * cp * sy - sr * sp * cy#
            q = Quaternion.normalize(q)
            return q
        else:
            q = Quaternion(0,0,0,0)
            cy = math.cos(yaw * 0.5)
            sy = math.sin(yaw * 0.5)
            cr = math.cos(roll * 0.5)
            sr = math.sin(roll * 0.5)
            cp = math.cos(pitch * 0.5)
            sp = math.sin(pitch * 0.5)
            q.w = cr * cp * cy + sr * sp * sy#
            q.x = sr * cp * cy - cr * sp * sy#
            q.y = cr * sp * cy + sr * cp * sy#
            q.z = cr * cp * sy - sr * sp * cy#
            return q

#################################################################################################################
#################################################################################################################
#################################################################################################################
class Pixel():
    def __init__(self, px):
        self.px = deepcopy(px)

#################################################################################################################
#################################################################################################################
#################################################################################################################
class Feature():
    def __init__(self, x, y, id=-1, global_position:NED=None):
        self.x = x
        self.y = y
        self.id = id
        self.counter = 0
        self.global_pos = global_position

    def __str__(self):
        return str(self.x)+","+str(self.y)+","+str(self.id)

#################################################################################################################
#################################################################################################################
#################################################################################################################
class Feature_List():
    def __init__(self, frame_id =0, ts=0):
        self.features = []
        self.features_dict = {}
        self.current_index = 0
        self.frame_id = frame_id
        self.ts = ts

#################################################################################################################
    def size(self):
        return len(self.features)
        
#################################################################################################################
    def append(self, feature:Feature):
        self.features.append(feature)
        self.features_dict[feature.id] = feature

#################################################################################################################
    def get_feature_by_id(self, id):
        if(id in self.features_dict):
            return True, self.features_dict[id]
        else:
            return False, None
    # def append_feature(self, feature:Feature):
    #     self.features.append(feature)
    #     # self.current_index = self.current_index + 1
        
#################################################################################################################
    def append_pt(self, x, y, ned_pos:NED=None):
        self.features.append(Feature(x,y,self.current_index, ned_pos))
        self.current_index = self.current_index + 1


#################################################################################################################
    def size(self):
        return len(self.features)

#################################################################################################################
    def __str__(self):
        string_out = ""
        # print(len(self.features))
        for feature in self.features:
            string_out = string_out + str(feature) + "\n"
        return string_out

#################################################################################################################
#################################################################################################################
#################################################################################################################
class Buffer():
    def __init__(self, size, data_is_timestamped = False):
        self._size = size
        self._next_index = 0
        self._data_list = []
        self._ts_list = []
        self._use_ts_list = data_is_timestamped
        

#################################################################################################################
    def add(self, data=None, ts=None, ):
        if(data is None):
            if(ts is not None):
                raise Exception("Only timestamp was provided, both data and timestamp must be provided as non None values") 
            else:
                raise Exception("No data was provided")
            # return False
        if(self._use_ts_list):
            if(ts is None):
                raise Exception("if using buffer with timestamped data timestamp must be given as input as non None value")
                # return False
        if(ts in self._ts_list):
            return False
        if(len(self._data_list) < self._size):
            self._data_list.append(data)
            if(self._use_ts_list):
                self._ts_list.append(ts)
        else:
            self._data_list[self._next_index] = data
        if(self._use_ts_list):
             self._ts_list[self._next_index] = ts
        self._next_index = self._increment_index(self._next_index)
        # print(self._increment_index(self._current_index))
        return True

#################################################################################################################
    def _increment_index(self, index):
        # print(index, self._size, len(self._ts_list), len(self._data_list))
        # if(index + 1 >= self._size):
        if(index + 1 >= len(self._data_list)):
            index = 0
        else:
            index = index + 1
        return index

#################################################################################################################
    def _decrement_index(self, index):
        if(index -1 < 0):
            # index = self._size-1
            index = len(self._data_list)-1
        else:
            index = index -1
        return index

#################################################################################################################
    def get_closest_data_to_ts(self, ts):
        if(not self._use_ts_list):
            return False, 0, 0
        if(len(self._ts_list)==0 or len(self._data_list)==0):
            return False, 0, 0
        current_index = self._decrement_index(self._next_index)
        prev_diff = 0
        first = True
        
        for i in range(0, len(self._ts_list)):
            diff = abs(self._ts_list[current_index] - ts)
            if(((diff >= prev_diff )and (not first) )or ts == self._ts_list[current_index]):
                break
            prev_diff = diff
            current_index = self._decrement_index(current_index)
            if(first):
                first = False
        current_index = self._increment_index(current_index)
        
        return True, self._ts_list[current_index], self._data_list[current_index]

#################################################################################################################
    def tail(self):
        return self._data_list[self._next_index]
#################################################################################################################
    def head(self):
        return self._data_list[self._next_index]
        
#################################################################################################################
    def get_buffer(self):
        if(self._use_ts_list):
            return self._data_list, self._ts_list
        else:
            return self._data_list

 #################################################################################################################
    def __str__(self):
        str_out = ""+str(self._next_index)+"\n"
        max_index = 0
        for i in range(0, len(self._ts_list)):
            str_out = str_out + str(self._ts_list[i])+":"+str(self._data_list[i])+"\n"
            max_index = i
        str_out = str_out + "\n"+str(max_index)
        return str_out

#################################################################################################################
#################################################################################################################
#################################################################################################################
class TimeUtils():
    @staticmethod
    def sleep_until(goal_time=None, start_time=None):
        if(start_time is None):
            TimeUtils.sleep(goal_time)
            return
        elapsed_time = TimeUtils.now()-start_time
        if(elapsed_time < goal_time):
                TimeUtils.sleep(goal_time-elapsed_time)
#################################################################################################################
    @staticmethod
    def sleep(sleep_time):
        time.sleep(sleep_time)
#################################################################################################################
    @staticmethod
    def now():
        # return time.time()
        return time.time()
#################################################################################################################
    @staticmethod
    def get_current_time_sec():
        # return time.time()
        return time.time()
#################################################################################################################
    @staticmethod
    def get_current_day_month_year_str():
        return  str(datetime.now()).split(' ')[0]
#################################################################################################################
    @staticmethod
    def get_current_time_hour_minute_sec_str(): #str(datetime.now()).split(' ')[1].split('.')[0].replace(':','_')
        hour_minute_sec_list = str(datetime.now()).split(' ')[1].split('.')
        hour_minute_sec = hour_minute_sec_list[0].replace(':','-')
        # milli_sec = hour_minute_sec_list[1][0:2]
        return hour_minute_sec
#################################################################################################################
    @staticmethod
    def get_utc_time_str():
        dt = datetime.now(timezone.utc)
        utc_time = dt.replace(tzinfo=timezone.utc) 
        return str(utc_time).split(" ")[1].split("+")[0]
#################################################################################################################
    @staticmethod
    def get_unique_datetime_str():
        day = TimeUtils.get_current_day_month_year_str()
        curr_time = TimeUtils.get_current_time_hour_minute_sec_str()
        return day+"_"+curr_time

#################################################################################################################
#################################################################################################################
#################################################################################################################
class Utils():

    @staticmethod
    def get_usb_path_for_device(device_str=None, device_substr = None):
        if(device_str is None and device_substr is None):
            return False, ""
        bash_script = """
        for sysdevpath in $(find /sys/bus/usb/devices/usb*/ -name dev); do
            (
                syspath="${sysdevpath%/dev}"
                devname="$(udevadm info -q name -p $syspath)"
                [[ "$devname" == "bus/"* ]] && exit
                eval "$(udevadm info -q property --export -p $syspath)"
                [[ -z "$ID_SERIAL" ]] && exit
                echo "/dev/$devname,$ID_SERIAL"
            )
        done
        """

        process = subprocess.Popen(['bash', '-c', bash_script], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

        output, error = process.communicate()
        # print(output)
        success = False
        device_path = ""
        if process.returncode == 0:
            output_list = output.split("\n")
            # search_id = "Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_f0b5f5aa5f83ed11a374ac5f9d1cc348"
            # print(output_list[0].split(","))
            for output in output_list:
                out_list = output.split(",")
                if(len(out_list) == 2):
                    _device_path, device_id = out_list[0],  out_list[1]
                else:
                    continue
                if(not device_str is None):
                    if(device_id == device_str):
                        success = True
                        device_path= _device_path
                        break
                elif(not device_substr is None):
                    if( device_id.find(device_substr)!= -1):
                        success = True
                        device_path= _device_path
                        break

        return success, device_path
#################################################################################################################
    @staticmethod
    def receive_from_queue(queue, queue_name, return_only_last =True):
        input_size = queue.qsize()+50
        data = None
        if(not return_only_last):
            data = []
        for _ in range(0, input_size):
            if(queue.qsize()==0):
                break
            try:
                _data = queue.get(timeout=QUEUE_READ_TIMEOUT)
                if(return_only_last):
                    data = data
                else:
                    data.append(_data)
            except Exception as e:
                # print("failed to retrieve data from "+str(queue_name)+" queue: "+str(e))
                break
        return data

#################################################################################################################
    @staticmethod
    def send_to_queue(queue, data, queue_name):
        try:
            queue.put_nowait(data)
        except:
            queue_size = queue.qsize() + 50
            for i in range(0, queue_size):
                if (queue.qsize()==0):
                    break
                try:
                    _ = queue.get(timeout=QUEUE_READ_TIMEOUT)
                except:
                    print("failed to get data from"+str(queue_name)+ "queue")
                    break
            try:
                queue.put_nowait(data)
            except:
                pass

#################################################################################################################
    @staticmethod
    def elev_az_to_vector(elev, az):
        x = np.tan(elev)
        y = np.tan(az)
        z = 1
        return x,y,z
#################################################################################################################
    @staticmethod
    def vector_to_elev_az(x,y,z):
        az = np.arctan2(y,z)
        elev = np.arctan2(x,z)
        return az, elev
#################################################################################################################
    @staticmethod
    def px_to_spherical(px_i, px_j):
        # mag = np.sqrt(px_i*px_i + px_j*px_j)
    
        r = np.sqrt(1-(px_j*px_j))
        d = np.sqrt((r*r)-(px_i*px_i))

        phi = np.arctan2(px_j,r)
        theta = np.arctan2(d, px_i)
        return theta, phi#, mag
    # x,y,z = Utils.spherical_to_cartesian(1, 0.1, 0.2)
    # print(Utils.cartesian_to_spherical(x,y,z))

#################################################################################################################
    @staticmethod
    def spherical_to_px(theta,phi):
        px_j = np.sin(phi)
        px_i = np.cos(theta)*np.sqrt(1-(px_j*px_j))
        return px_i, px_j

#################################################################################################################
    @staticmethod
    #theta = azimuth, phi = elevation
    def spherical_to_cartesian(p, theta, phi):
        # print("input to sphereical to cartesian p,theta,phi",p,theta,phi)
        # phi = elev
        # theta = az
        #phi is angle between R and xy plane - z is up, x is forward, y is left
        # theta is angle between projection of R on the XY plane, 0 is straight on x axis, 90 is straight on y axis
        x = p*np.cos(phi)*np.cos(theta)
        y = p*np.cos(phi)*np.sin(theta)
        z = p*np.sin(phi)
        # z = p*np.sin(phi)
        # r = np.sqrt((p*p) -(z*z))
        # x = r*np.cos(theta)
        # y = r*np.sin(theta)
        return x,y,z
        
#################################################################################################################
    def cartesian_to_spherical(x,y,z):
        p = np.sqrt(x*x + y*y + z*z)
        theta = np.arctan2(y,x)
        phi = np.arccos(z/p)
        # phi = np.arctan2(z,np.sqrt((x*x)+(y*y)))
        # theta = Utils.angle_diff_rad(np.radians(90),theta)
        return p, theta, phi

#################################################################################################################
    @staticmethod
    def get_log_dir():
        log_dir = ""
        success = False
        path = None
        for name in USB_DISK_NAME_LIST:
            success, path = Utils.get_usb_path_for_device(device_substr=name)
            if(success):
                break
        if(success):
            subprocess.getoutput(['source scripts/usb_mount.sh'])
            parent = "/mnt/usb/"
        else:
            parent = (os.path.abspath(os.path.join( os.getcwd(), os.pardir)))
        day = TimeUtils.get_current_day_month_year_str()
        curr_time = TimeUtils.get_current_time_hour_minute_sec_str()
        # log_dir = os.path.join( os.getcwd(), '..', 'logs',str(day),str(curr_time)  )
        log_dir = os.path.join( parent, 'logs',str(day),str(curr_time))
        if(not os.path.exists(log_dir)):
            os.makedirs(log_dir)    
        out = subprocess.run([os.getcwd()+"/scripts/"+'auto_update.sh', str(log_dir)], check=True)    
        print("out",out)
        return log_dir

#################################################################################################################
    @staticmethod
    def rotate(origin_x, origin_y, px, py, angle_rad):

        qx = origin_x + math.cos(angle_rad) * (px - origin_x) - math.sin(angle_rad) * (py - origin_y)
        qy = origin_y + math.sin(angle_rad) * (px - origin_x) + math.cos(angle_rad) * (py - origin_y)
        return qx, qy

#################################################################################################################
    @staticmethod
    def angle_diff_deg(x_deg, y_deg):
        x = np.radians(x_deg)
        y = np.radians(y_deg)
        return np.degrees(math.atan2(math.sin(x-y), math.cos(x-y)))

#################################################################################################################
    @staticmethod
    def angle_diff_rad(x, y):
        return math.atan2(math.sin(x-y), math.cos(x-y))
#################################################################################################################
    @staticmethod
    def rolling_avg(old_data, new_data, window_size):
        alpha = 1/window_size
        return (old_data * (1-alpha)) + (new_data * alpha)
        return old_data
#################################################################################################################
    @staticmethod
    def calc_2d_dist( x1, y1,x2, y2):
        dx = x1 - x2
        dy = y1- y2
        return np.sqrt( (dx*dx) + (dy*dy))
#################################################################################################################
    @staticmethod
    def normalize(val,upper_limit, lower_limit, clip=True ):
        if(clip):
            val= Utils.clip(val, lower_limit, upper_limit)
        return val * ((upper_limit -lower_limit ) + lower_limit)
#################################################################################################################
    @staticmethod
    def clip(val, lower_limit, upper_limit):
        if(val > upper_limit):
            return upper_limit
        if(val < lower_limit):
            return lower_limit
        return val
#################################################################################################################
BIN = "BIN"
TEXT = ""  
PKL = "PKL"    
CSV = "CSV"
MUTEX_TIMEOUT_SEC = 0.5  
COUNT_BETWEEN_FLUSH_BIN = 10

#################################################################################################################
#################################################################################################################
#################################################################################################################
class Logger():
    
    def __init__(self, log_name, log_dir, save_log_to_file, print_logs_to_console, datatype = TEXT, suffix_in=None):
        self.record_log = save_log_to_file
        self._datatype = datatype
        self._field_names = []
        self._count_since_last_flush = 0
        self.log_name = log_name
        self._field_name_line_exists = False
        self.print_logs_to_console = print_logs_to_console
        self.file_mutex = Lock()
        # signal.signal(signal.SIGINT, self._ctrl_c_handler)
        try:
            self.file_mutex.acquire(timeout=MUTEX_TIMEOUT_SEC)
            if(self.record_log):
                suffix = ".txt"
                file_write_type = "w"
                if(self._datatype==CSV):
                    suffix = ".csv"
                    file_write_type = "w"
                elif(self._datatype == PKL):
                    suffix = ".pkl"
                    file_write_type = "wb"
                elif(self._datatype == BIN):
                    suffix = ".bin"
                    file_write_type = "wb"
                else:
                    suffix = ".txt"
                    file_write_type = "w"
                if not os.path.exists(log_dir):
                    os.makedirs(log_dir)
                if(suffix_in is not None):
                    suffix = "."+suffix_in
                file_name = str(log_dir)+"/"+str(log_name)+suffix
                if(self._datatype == BIN):
                    self.file = open(file_name, file_write_type)
                else:
                    # print("opening file:", file_name)
                    self.file = open(file_name, file_write_type)
                # print(self.file)
            self.file_mutex.release()
        except Exception as e:
            print("mutex timed out in logger:"+str(e))
            self.record_log = False
            try:
                self.file_mutex.release()
            except Exception as e:
                print(Utils.make_exception_msg("error releasing in init"))

#################################################################################################################

    def write(self, data):
        try:
            self.file_mutex.acquire(timeout=MUTEX_TIMEOUT_SEC)
            self.file.write(data)
            if(self._datatype == BIN or 1):
                
                if(self._count_since_last_flush > COUNT_BETWEEN_FLUSH_BIN):
                    # print(self._count_since_last_flush, "flushing")
                    self._count_since_last_flush = 0
                    self.file.flush()
                else:
                    # print(self._count_since_last_flush, "not flushing")
                    self._count_since_last_flush = self._count_since_last_flush + 1
            else:
                self.file.flush()
            self.file_mutex.release()
        except Exception as e:
            print("mutex timed out in logger in write:"+str(e))
            try:
                self.file_mutex.release()
            except Exception as e:
                 print(Utils.make_exception_msg("error releasing in write"))

 #################################################################################################################
    def log(self, data):
        try:
            if(self.record_log):
                if(self._datatype == CSV):
                    if(isinstance(data, dict)):
                        if(not self._field_name_line_exists):
                            self._field_name_line_exists = True
                            first = True
                            field_count = 0
                            for key in data:
                                field_count = field_count + 1
                                self._field_names.append(key)
                                if(first):
                                    self.write(str(key))
                                    first = False
                                else:
                                    self.write(","+str(key))
                            self._field_count = field_count
                            self.write("\n")
                        first = True
                        
                        for key in self._field_names:
                            if(first):
                                self.write(str(data[key]))
                                first = False
                            else:
                                self.write(","+str(data[key]))
                        self.write("\n")
                    else:
                        self.write("ERROR: data must be passed as dict")
                elif(self._datatype == PKL):
                    pickle.dump(data, self.file)
                elif(self._datatype == BIN):
                    self.write(data)
                    return
                else:
                    data_out = ""
                    if(type(data) is list):
                        first = True
                        for val in data:
                            if(first):
                                first = False
                                data_out = str(val)
                            else:
                                data_out = data_out+","+str(val)
                        data = data_out
                    self.write(str(data)+"\n")
            if(self._datatype != BIN):
                if(self.print_logs_to_console):
                    print(str(data))
        except Exception as e:
            print(Utils.make_exception_msg("logger failed"))

#################################################################################################################
    def log_err(self, data):
        if(self._datatype == BIN or self._datatype == PKL or self._datatype == CSV):
            return
        data = str("ERROR: "+str(data))
        # if(self.print_logs_to_console):
        Utils.print_red(data)
        if(self.record_log):
            # print("writing error msg "+str(data))
            self.write(str(data)+"\n")

#################################################################################################################
    def close(self):
        if(self.record_log):
            if(self.print_logs_to_console):
                print("closing logger")
            try:
                self.file_mutex.acquire(timeout=MUTEX_TIMEOUT_SEC)
                self.file.flush()
                self.file.close()
                self.file_mutex.release()
            except Exception as e:
                print("mutex timed out in logger:"+str(e))
                try:
                    self.file_mutex.release()
                except Exception as e:
                     print(Utils.make_exception_msg("error releasing in close"))
                self.file.close()
        self.record_log = False
#################################################################################################################
#################################################################################################################
#################################################################################################################
def limitInclination(maxAngle, thrustVector):
    thrustDir = thrustVector/np.linalg.norm(thrustVector)
    if arccos(thrustDir[2]) > maxAngle:
        thrustDir[2] = np.sqrt(thrustDir[0]**2+thrustDir[1]**2)/tan(maxAngle)
        thrustDir = thrustDir/np.linalg.norm(thrustDir)
        newThrust = np.dot(thrustDir, thrustVector)

    return thrustDir*newThrust
    
#################################################################################################################
def lissajous_func(t, A=1, B=1, C=0.2, a=2, b=3, c=2, alt=-1, w = 2 * pi / 10):
        # Lissajous curve
    # x=A*sin(a*t+d), y=B*sin(b*t), z=alt+C*cos(c*t)
    # several common parameters:
    # a:b   d    0       pi/4      pi/2      3/4*pi    pi
    # 1:1        /      ellipse   circle    ellipce    \
    # 1:2        )      8-figure  8-figure  8-figure   (
    # 1:3        S      

    d = pi / 2 * 0

    # % t = linspace(0, 2*pi, 2*pi*100+1);
    # % x = A * sin(a * t + d);
    # % y = B * sin(b * t);
    # % z = alt + C * cos(2 * t);
    # % plot3(x, y, z);

    x = np.array([A * sin(a * t + d), B * sin(b * t), alt + C * cos(c * t)])
    x_dot = np.array([A * a * cos(a * t + d), B * b * cos(b * t), C * c * -sin(c * t)])
    x_2dot = np.array([A * a**2 * -sin(a * t + d), B * b**2 * -sin(b * t), C * c**2 * -cos(c * t)])
    x_3dot = np.array([A * a**3 * -cos(a * t + d), B * b**3 * -cos(b * t), C * c**3 * sin(c * t)])
    x_4dot = np.array([A * a**4 * sin(a * t + d), B * b**4 * sin(b * t), C * c**4 * cos(c * t)])

    b1 = np.array([cos(w * t), sin(w * t), 0])
    b1_dot = w * np.array([-sin(w * t), cos(w * t), 0])
    b1_2dot = w**2 * np.array([-cos(w * t), -sin(w * t), 0])
    
    return (x, x_dot, x_2dot, x_3dot, x_4dot), (b1, b1_dot, b1_2dot)
            
#################################################################################################################
def plot3DcoordMat(p,R, linewidth=2, ax = None):
    axisx = np.array([[0,0,0],[1,0,0]])
    axisy = np.array([[0,0,0],[0,1,0]])
    axisz = np.array([[0,0,0],[0,0,1]])
    axisx = R@axisx.T
    axisy = R@axisy.T
    axisz = R@axisz.T
    
    if(ax is None):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
    
    ax.quiver(p[0], p[1], p[2], axisx[0], axisx[1], axisx[2], color='r',linewidth=linewidth)
    ax.quiver(p[0], p[1], p[2], axisy[0], axisy[1], axisy[2], color='g',linewidth=linewidth)
    ax.quiver(p[0], p[1], p[2], axisz[0], axisz[1], axisz[2], color='b',linewidth=linewidth)
    return ax

def plot3Dcoord(p,q, linewidth=2, ax = None):
    R = q.to_rotation_matrix()
    return plot3DcoordMat(p,R, linewidth, ax)

#################################################################################################################       
def quiver3D(vec, pos=np.zeros(3), linewidth=2, ax = None, color='r'):   
    if(ax is None):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.axis('equal')
    
    ax.quiver(pos[0], pos[1], pos[2], vec[0], vec[1], vec[2], color=color, linewidth=linewidth)
    return ax

#################################################################################################################   
def plot3DBoxMat(p, R, length=1, wide=0.7, height=0.3, linewidth=2, ax = None, axisLabels = False):
    corners=np.array([[length/2, wide/2, 0], 
                      [length/2, -wide/2, 0], 
                      [-length/2, -wide/2, 0], 
                      [-length/2, wide/2, 0], 
                      [length/2, wide/2, height], 
                      [length/2, -wide/2, height], 
                      [-length/2, -wide/2, height], 
                      [-length/2, wide/2, height]])
    corners = corners.T
    corners = R@corners
    corners = corners + np.array([p]).T@np.ones((1,8))
    if(ax is None):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        
    ax.plot([corners[0,0], corners[0,1], corners[0,2], corners[0,3], corners[0,0]], 
            [corners[1,0], corners[1,1], corners[1,2], corners[1,3], corners[1,0]], 
            [corners[2,0], corners[2,1], corners[2,2], corners[2,3], corners[2,0]], 'b',linewidth=linewidth)
    ax.plot([corners[0,4], corners[0,5], corners[0,6], corners[0,7], corners[0,4]],
            [corners[1,4], corners[1,5], corners[1,6], corners[1,7], corners[1,4]],
            [corners[2,4], corners[2,5], corners[2,6], corners[2,7], corners[2,4]], 'r',linewidth=linewidth)
    ax.plot([corners[0,0], corners[0,4]],
            [corners[1,0], corners[1,4]],
            [corners[2,0], corners[2,4]], 'k',linewidth=linewidth*3)
    ax.plot([corners[0,1], corners[0,5]],
            [corners[1,1], corners[1,5]],
            [corners[2,1], corners[2,5]], 'k',linewidth=linewidth*3)
    ax.plot([corners[0,2], corners[0,6]],
            [corners[1,2], corners[1,6]],
            [corners[2,2], corners[2,6]], 'b',linewidth=linewidth)
    ax.plot([corners[0,3], corners[0,7]],
            [corners[1,3], corners[1,7]],
            [corners[2,3], corners[2,7]], 'b',linewidth=linewidth)
    if(axisLabels):
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        ax.axis('equal')

    return ax
    
#################################################################################################################
def plot3DBox(p, q, length=1, wide=0.7, height=0.3, linewidth=2, ax = None, axisLabels = False):
    R = q.to_rotation_matrix()
    return plot3DBoxMat(p, R, length, wide, height, linewidth, ax, axisLabels)

#################################################################################################################
def plotCamera(fov_vec, pos_ned, quat_ned_cam, height=1, linewidth=2, ax = None, axisLabels = False):
    corners = np.array([[0,0,0],
                      [height*tan(fov_vec[0]/2), height*tan(fov_vec[1]/2), -height],
                      [height*tan(fov_vec[0]/2), -height*tan(fov_vec[1]/2), -height],
                      [-height*tan(fov_vec[0]/2), -height*tan(fov_vec[1]/2), -height],
                      [-height*tan(fov_vec[0]/2), height*tan(fov_vec[1]/2), -height]])
    corners = corners.T
    R = quat_ned_cam.to_rotation_matrix()
    corners = R@corners
    if(ax is None):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.axis('equal')

    corners = corners + np.array([pos_ned]).T@np.ones((1,5))
    ax.plot([corners[0,1], corners[0,2]], [corners[1,1], corners[1,2]], [corners[2,1], corners[2,2]], 'k',linewidth=linewidth)
    ax.plot([corners[0,2], corners[0,3]], [corners[1,2], corners[1,3]], [corners[2,2], corners[2,3]], 'k',linewidth=linewidth/2)
    ax.plot([corners[0,3], corners[0,4]], [corners[1,3], corners[1,4]], [corners[2,3], corners[2,4]], 'k',linewidth=linewidth)
    ax.plot([corners[0,4], corners[0,1]], [corners[1,4], corners[1,1]], [corners[2,4], corners[2,1]], 'k',linewidth=linewidth)
              
    ax.plot([corners[0,0], corners[0,1]], [corners[1,0], corners[1,1]], [corners[2,0], corners[2,1]], 'b',linewidth=linewidth)
    ax.plot([corners[0,0], corners[0,2]], [corners[1,0], corners[1,2]], [corners[2,0], corners[2,2]], 'b',linewidth=linewidth)
    ax.plot([corners[0,0], corners[0,3]], [corners[1,0], corners[1,3]], [corners[2,0], corners[2,3]], 'b',linewidth=linewidth)
    ax.plot([corners[0,0], corners[0,4]], [corners[1,0], corners[1,4]], [corners[2,0], corners[2,4]], 'b',linewidth=linewidth)
    if(axisLabels):
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        ax.axis('equal')
    return ax
#################################################################################################################
def ray_plane_intersection(ray_origin, ray_direction, plane_coeffs):
    """
    Find the intersection of a ray with a plane in 3D.
    
    :param ray_origin: (numpy array) A point on the ray [x0, y0, z0].
    :param ray_direction: (numpy array) The direction of the ray [dx, dy, dz].
    :param plane_coeffs: (numpy array) The plane coefficients [A, B, C, D] for Ax + By + Cz + D = 0.
    :return: (numpy array or None) The intersection point [x, y, z] or None if no intersection.
    """
    # A, B, C, D = plane_coeffs
    # x0, y0, z0 = ray_origin
    # dx, dy, dz = ray_direction
    
    denominator = np.dot(plane_coeffs[0:3],ray_direction)     # A * dx + B * dy + C * dz
    
    if abs(denominator) < 1e-6:
        return None  # The ray is parallel to the plane
    if denominator>0:
        return None  # the ray is in the same direction with plane normal
    
    t = -(np.dot(ray_origin,plane_coeffs[0:3]) + plane_coeffs[3])/denominator   # (distance to the plane)/(cos(angle with plane normal)) = 
                                                                            # -(A * x0 + B * y0 + C * z0 + D) / denominator      
    intersection = ray_origin + t * ray_direction
    return intersection
#################################################################################################################
def unitVec(vec):
    norm = np.linalg.norm(vec)
    if norm == 0:
        return vec
    return vec / norm
#################################################################################################################
## main section for testing
if __name__ == "__main__":
    vector = np.array([1,2,3])
    vectornew = limitInclination(np.deg2rad(10), vector)
    
    # plot two vectors
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.quiver(0, 0, 0, vector[0], vector[1], vector[2], color='r', linewidth=2)
    ax.quiver(0, 0, 0, vectornew[0], vectornew[1], vectornew[2], color='b', linewidth=2)
    ax.quiver(0, 0, 0, 0, 0, 1, color='b', linewidth=2)
    
    plt.axis('equal')    
    ax.set_xlim([-3, 3])
    ax.set_ylim([-3, 3])
    ax.set_zlim([-3, 3])
    # axis equal
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    

    plt.show()
    i=1
    