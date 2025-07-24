#!/bin/python3

import time
import numpy as np
from numpy import pi
from common import *
# from scipy.spatial.transform import Rotation as R
from matplotlib import pyplot as plt
from guidance import Guidance, GUIDANCE_TYPE
# from config_parser import Config_Parser
from copy import deepcopy

from enum import Enum

import os

"""
python sim:
target: start pt, velocity vector
interceptor: start pt, velocity vector
use true pronav
step:
    update target pos from vector
    update




tuning:
1) start with setting roll pitch to 0 and get the stationary tracking height loop working
     -within that loop start by setting the acceleration to 0.5 and tuning to that, then setting vel
2) after that set thrust to constant to tune pitch loop in the same way, starting with accel, then vel, then tracker
delta_flight_path_angle = N * delta LOS angle

TODO:
 -maybe better to use simpler navigation tactics so that we can keep the image centered on the target
 -add filter to line of sight-maybe take distance between prev los vel and new los_vel, if diff > max_diff, new los vel = prev los vel + max_diff
 -add greater delay to images
"""
GRAVITYACCL=-9.80665
ADD_NOISE = False
MAX_THRUST_KG = 2.14
PITCHLIMITDEG = 10
ROLLLIMITDEG = 10
MAXIMALVELOCITY = 10
MAXIMALACCELERATION = 100

# accel = N * V_closing * lambda_dot
class HOMING_STAGE(Enum):
    NONE = 0
    SECTION = 1
    VELOCITY = 2
    PURSUIT = 3
    VELOCITY_ENDGAME = 4


###############################################################################################################################################
###############################################################################################################################################
###############################################################################################################################################
class Control():
    def __init__(self, config_dir, log_directory, controller):   # image size in pixels (width, height)

        self._start_vertical_los_deg = None
        self._noise_sim = None
        ############################## Parameters ################################
        # system_config_parser = Config_Parser(path=os.path.join(config_dir, "system_config.json"))
        # vehicle_config_file_name = system_config_parser.get_value("vehicle_config_file", default_value="")
        vehicle_data_parser = None
        # if(vehicle_config_file_name is not None):
        #     vehicle_data_parser = Config_Parser(path=os.path.join(config_dir,"vehicle", vehicle_config_file_name), save_copy=True, output_directory=log_directory)
        if(vehicle_data_parser is None):
            print("config init failed")            
        
        self._min_thrust = 0.01
        self._max_thrust = 1
        
        # GUIDANCE
        guidance_constant_N = 8 
      
        self.MaximalThrust = controller.param.mass*9.81*2.5
        # self._pitch_accel_sp_p = 0.05
        self.enableIntegrator = False
                
        #########################################################################
        self.controlnode = controller

        self._accel_cmd_ned = None

        self._control_logger = Logger("control_logs", log_directory, save_log_to_file=True, print_logs_to_console=False, datatype="CSV") 
        # self._control_input_logger = Logger("control_input", log_directory, save_log_to_file=True, print_logs_to_console=False, datatype="CSV")       
       
        # self._current_error_horizontal_rad = 0
        self._start_alt = None
      
        self._last_pitch_update = time.time()

        self._finished_stationary_tracking = False

        self._current_pos_ned = np.zeros(3)
        self._current_vel_ned = np.zeros(3)
 
        self.prev_los_distance = None
        self.prev_los_ned_dir = None
        
        guidance_type = GUIDANCE_TYPE.PURE_PURSUIT                         #max_accel_cmd_ms_s_s=guidance_horizontal_accel_cutoff
        self._guidance = Guidance(N=guidance_constant_N, type=guidance_type) #60 for fast interceptions

        
        #x = forward/back, y = left/right
        ### currently unused eventually should be used to stay in the same position without gps before starting homing

###############################################################################################################################################
    def get_cmd(self, pos_ned, vel_ned, accel_ned, gyro_ned, 
                        quat_ned_bodyfrd, imu_ts, step_dt, current_ts,
                        counter, trajDest_ned, 
                        headingDest=None, controlType=None, log_data=True,
                        homingStage=None):

        b1d_dot = np.zeros(3)
        b1d_ddot = np.zeros(3)  
        if controlType is None:
            pos_control = True
            vel_control = False
            controlType = [pos_control, vel_control, YAW_COMMAND.NO_CONTROL]
            
        estimated_tar_pos_ned = trajDest_ned[0]
        pos_des_ned = trajDest_ned[0] 
        vel_des_ned = trajDest_ned[1]
        accel_des_ned = trajDest_ned[2]
        
        b1d_ned = headingDest[0] if headingDest is not None else np.array([1,0,0]) #quat_ned_bodyfrd.rotate_vec([1,0,0])
        if headingDest is not None:
            b1d_dot = headingDest[1]
            b1d_ddot = headingDest[2]

                
        currentBodyState = (pos_ned, vel_ned, accel_ned, gyro_ned, quat_ned_bodyfrd)

        desiredBodyState = [(pos_des_ned, vel_des_ned, accel_des_ned, np.zeros(3), np.zeros(3)), # desired position, velocity and acceleration (taken from trajectory)
                        (b1d_ned, b1d_dot, b1d_ddot)]   # desired direction of the first body axis (taken from trajectory)
        
        f_total, R_desired, Omega_desired_frd = self.controlnode.getCommand(currentBodyState, desiredBodyState, controlType)
        # f_total_ref, R_desired_ref, Omega_desired_frd_ref = self.controlnode_ref.getCommand(currentBodyState, desiredBodyState, controlType)

        if self.controlnode.controllerType == "VelocityPID":
            velCmdDir = f_total/np.linalg.norm(f_total)
            velCmdAbs = np.linalg.norm(f_total)
            velCmdAbsClipped = np.clip(velCmdAbs, 0, MAXIMALVELOCITY)
            velCmd = velCmdDir * velCmdAbsClipped
            command = velCmd
        elif self.controlnode.controllerType == "AccelerationPID":
            accCmdDir = f_total/np.linalg.norm(f_total)
            accCmdAbs = np.linalg.norm(f_total)
            accCmdAbsClipped = np.clip(accCmdAbs, 0, MAXIMALACCELERATION)
            accCmd = accCmdDir * accCmdAbsClipped
            command = accCmd
        else:
            command = np.min([f_total/(self.MaximalThrust*.7), self._max_thrust])
            command = np.max([command, self._min_thrust])
        
        quat_ned_desbodyfrd = Quaternion.from_matrix(R_desired)

        rpyRate_cmd = Omega_desired_frd

        if(log_data):              
            self.log_control_data(command=command, rpy_rate_cmd=rpyRate_cmd, quat_ned_desbodyfrd_cmd=quat_ned_desbodyfrd,Omega_desired_frd=Omega_desired_frd,
                              current_pos_ned=self._current_pos_ned, cur_vel_ned=self._current_vel_ned, 
                              gyro_ned=gyro_ned, accel_ned=accel_ned, quat_ned_bodyfrd=quat_ned_bodyfrd,
                              est_tar_pos_ned=estimated_tar_pos_ned, vel_des_ned=vel_des_ned, imu_ts=imu_ts, dt=step_dt, current_ts=current_ts, counter=counter)
        
        return command, rpyRate_cmd, quat_ned_desbodyfrd
       
###############################################################################################################################################
    def rotate2cameraDirection(self, quat_ned_desbodyfrd, quat_bodyfrd_cam, los_ned_dir, plotFig=False):
        """
        Rotate the desired quaternion around desbodyfrd Z axis (retaining the thrust Z FRD direction),
        in order to align the desired camera direction with the LOS direction as close as possible.
        """
        quat_desbodyfrd_cam = quat_bodyfrd_cam  # the transformatiin from bodyfrd to camera and from desbody frame to camera frame is the same
        rotAxisZfrd_ned = quat_ned_desbodyfrd.rotate_vec([0,0,1])
        rotAxisZfrd_ned_dir = rotAxisZfrd_ned/np.linalg.norm(rotAxisZfrd_ned)
        camZ_ned = (quat_ned_desbodyfrd*quat_desbodyfrd_cam).rotate_vec([0,0,-1])
        camZ_ned_dir = camZ_ned/np.linalg.norm(camZ_ned)

        if plotFig:
            ax=quiver3D(los_ned_dir, color='g')
            ax=quiver3D(rotAxisZfrd_ned_dir, ax=ax,color='g')
            ax=quiver3D(camZ_ned_dir, ax=ax,color='r')

        rotAngleArr = np.linspace(0,360,360)
        los_cam_angle = np.zeros(len(rotAngleArr))
        for i in range(len(rotAngleArr)):
            rotAngle = np.deg2rad(rotAngleArr[i])
            quat_Z_axis_rot_ned = Quaternion.from_axis_angle(axis=rotAxisZfrd_ned_dir, angle=rotAngle)
            camZ_rotated_ned_dir = quat_Z_axis_rot_ned.rotate_vec(camZ_ned_dir)
            los_cam_angle[i] = np.arctan2(np.linalg.norm(np.cross(camZ_rotated_ned_dir, los_ned_dir)), np.dot(camZ_rotated_ned_dir, los_ned_dir))
            if plotFig and i%20==0:
                ax=quiver3D(camZ_rotated_ned_dir, ax=ax,color='b')
 
        rotAngle = np.deg2rad(np.argmin(los_cam_angle))
        quat_Z_axis_rot_ned = Quaternion.from_axis_angle(axis=rotAxisZfrd_ned_dir, angle=rotAngle)
        quat_Z_axis_rot_ned = -quat_Z_axis_rot_ned if (quat_Z_axis_rot_ned.dot(quat_ned_desbodyfrd) < 0) else quat_Z_axis_rot_ned
        
        rotAxisZfrd_ned2=(quat_Z_axis_rot_ned*quat_ned_desbodyfrd).rotate_vec([0,0,1])
        camZ_rotated_ned =        (quat_Z_axis_rot_ned*quat_ned_desbodyfrd*quat_desbodyfrd_cam).rotate_vec([0,0,-1])
        quat_ned_desbodyfrd_new = (quat_Z_axis_rot_ned*quat_ned_desbodyfrd)
        return quat_ned_desbodyfrd_new
###############################################################################################################################################
    def log_control_data(self, command, rpy_rate_cmd, quat_ned_desbodyfrd_cmd,Omega_desired_frd,
                              current_pos_ned, cur_vel_ned,  
                              gyro_ned, accel_ned, quat_ned_bodyfrd,
                              imu_ts, dt, current_ts, counter, est_tar_pos_ned = np.array([0,0,0]), vel_des_ned = np.array([0,0,0])):
        self._control_logger.log({"command":command, "roll_rate_cmd":rpy_rate_cmd[0], "pitch_rate_cmd":rpy_rate_cmd[1], "yaw_rate_cmd":rpy_rate_cmd[2],
                                  "quat_ned_desbodyfrd_cmd_x":quat_ned_desbodyfrd_cmd.x, "quat_ned_desbodyfrd_cmd_y":quat_ned_desbodyfrd_cmd.y,
                                  "quat_ned_desbodyfrd_cmd_z":quat_ned_desbodyfrd_cmd.z, "quat_ned_desbodyfrd_cmd_w":quat_ned_desbodyfrd_cmd.w,
                                  "current_pos_ned_x":current_pos_ned[0], "current_pos_ned_y":current_pos_ned[1], "current_pos_ned_z":current_pos_ned[2],
                                  "cur_vel_ned_x":cur_vel_ned[0], "cur_vel_ned_y":cur_vel_ned[1], "cur_vel_ned_z":cur_vel_ned[2],
                                  "gyro_ned_x":gyro_ned[0], "gyro_ned_y":gyro_ned[1], "gyro_ned_z":gyro_ned[2],
                                  "accel_ned_x":accel_ned[0], "accel_ned_y":accel_ned[1], "accel_ned_z":accel_ned[2],
                                  "Omega_desired_frd_x":Omega_desired_frd[0], "Omega_desired_frd_y":Omega_desired_frd[1], "Omega_desired_frd_z":Omega_desired_frd[2],
                                  "quat_ned_bodyfrd_x":quat_ned_bodyfrd.x, "quat_ned_bodyfrd_y":quat_ned_bodyfrd.y,
                                  "quat_ned_bodyfrd_z":quat_ned_bodyfrd.z, "quat_ned_bodyfrd_w":quat_ned_bodyfrd.w,
                                  "est_tar_pos_ned_x":est_tar_pos_ned[0], "est_tar_pos_ned_y":est_tar_pos_ned[1], "est_tar_pos_ned_z":est_tar_pos_ned[2],
                                  "vel_des_ned_x":vel_des_ned[0], "vel_des_ned_y":vel_des_ned[1], "vel_des_ned_z":vel_des_ned[2],
                                  "imu_ts":imu_ts, "dt":dt, "current_ts":current_ts, "counter":counter})

###############################################################################################################
###############################################################################################################################################
###############################################################################################################################################
###############################################################################################################################################
###############################################################################################################################################
###############################################################################################################################################
###############################################################################################################################################
if __name__ == '__main__':
    control = Control(100,100)
    quat_ned_bodyfrd = Quaternion.euler_to_quat(Euler(roll=np.radians(0),pitch=np.radians(-35),yaw=np.radians(40)))
    # control._calc_vertical_thrust(roll=np.radians(0), pitch=np.radians(0))
    thrust_vector = np.array([0,0,1])
    accel_ned[0] = -5.5
    accel_ned[1] = -0.1
    accel_ned[2] = -8.1
    accel_ned = quat_ned_bodyfrd.rotate_vector(x=accel_ned[0], y=accel_ned[1], z=accel_ned[2])
    print(accel_ned)
    #ned to body is reverse_rotate is active rotate
    active = quat_ned_bodyfrd.active_rotate_vector(thrust_vector[0], thrust_vector[1], thrust_vector[2])
    passive = quat_ned_bodyfrd.passive_rotate_vector(thrust_vector[0], thrust_vector[1], thrust_vector[2])
    reverse = quat_ned_bodyfrd.reverse_rotate_vector(thrust_vector[0], thrust_vector[1], thrust_vector[2])
    rotate = quat_ned_bodyfrd.rotate_vector(thrust_vector[0], thrust_vector[1], thrust_vector[2])
    # reverse = vehicle_orientaion_quat.reverse_rotate_vector(out[0], out[1], out[2])
    print("out    ",reverse)
    print("active ", active) 
    print("rotate ", rotate)
    print("passive", passive)
    # print("passive", passive)
    # out_quat = control._accel_cmd_to_orientation(accel_horiz=2, accel_vertical=0)
    # print(out_quat)
    # euler_out = (Quaternion.quat_to_euler(out_quat))
    # print("roll", np.degrees(euler_out.rpy[0]), "pitch",np.degrees(euler_out.rpy[1]), "yaw",np.degrees(euler_out.rpy[2]))
    # v1 = [1,35,-5]
    
    # v2 = [4,-2,-1]
    # fpa = control._get_thrust_horizontal_direction(vehicle_orientaion_quat)
    # print("fpa: ",np.degrees(fpa))
"""
roll=0, pitch=45, yaw = 
orientation: 
  x: 1.875534758779368e-06
  y: -0.341619257556323
  z: 8.72018295690404e-05
  w: -0.939838433356369

"""


            # if los_ned_dir[2] > 0:  # using assumption that the target is on the ground
            #     estimated_tar_pos_ned = pos_ned + los_ned_dir * np.abs(pos_ned[2]/los_ned_dir[2])
            #     estimated_range_to_target = np.abs(self._current_pos_ned[2]/los_ned_dir[2])  
            # else:
            #     estimated_tar_pos_ned = np.array([0,0,0])
            #     estimated_range_to_target = np.inf

            # pos_des_ned = None
            # b1d_ned = quat_ned_bodyfrd.rotate_vec([1,0,0]) #los_ned_dir

            # if (np.dot(vel_ned, los_ned_dir) - PURSUIT_SPEED > 0 and self.homing_stage == HOMING_STAGE.INITIAL):
            #     self.homing_stage = HOMING_STAGE.FINAL
                
            # if self.homing_stage == HOMING_STAGE.FINAL:
            #     self._accel_cmd_ned, vel_des_ned = self._guidance.guidance_step(vrel_ned=-elf._current_vel_ned, 
            #                                                                     los_ned_dir=los_ned_dir, 
            #                                                             step_dt=step_dt, flight_path_angle_rad=0, 
            #                                                             range_to_target=estimated_range_to_target, 
            #                                                             prev_los_ned_dir=self.prev_los_ned_dir)
            #     vel_des_ned = vel_des_ned/np.linalg.norm(vel_des_ned) * PURSUIT_SPEED
            # else:
            #     vel_des_ned = los_ned_dir * PURSUIT_SPEED
            #     self._accel_cmd_ned = np.array([0,0,0])
            
            # accel_des_ned = self._accel_cmd_ned
