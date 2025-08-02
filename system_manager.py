#!/bin/python3
import time
#from hardware_adapter import Hardware_Adapter
#from virtual_tracker import Virtual_Tracker
from common import *
import numpy as np
# from config_parser import Config_Parser
from control import *
# from tracker import Tracker
import os
import pymap3d
# import cv2
import sys
import zmq
# from Filter.AllInOneKalman import AllInOneKalman
# from virtual_tracker import Tracker

# from controlGeom import GeometricController
# from controlGeomAdaptive import AdaptiveGeometricController
# from controlBrescianini import BrescianiniController
# from controlKooijman import KooijmanController
from controlVelocityPID import VelocityPIDController
# from controlAccelerationPID import AccelerationPIDController
# from controlJaeyoung import JaeyoungController

sys.path.append('../')
try:
    import src.zmqTopics as zmqTopics
    import src.zmqWrapper as zmqWrapper
    import src.mps as mps
except:
    import zmqTopics
    import zmqWrapper
    import mps

class MISSION_TYPE(Enum):
    NONE = 0
    WAYPOINT = 1
    VELOCITY = 2
    CIRCLE = 3
    LISSAJOUS = 4
    TRACKER = 5
    SECTION = 6
#################################################################################################################
GOAL_LOOP_FREQ_HZ = 50
GPS_SEARCH_TIMEOUT_SEC = 3
#################################################################################################################
class System_Manager():
    def __init__(self, config_dir, log_dir, sim_object=None, external_imu=None, use_usb_for_mavlink=False):
        self._overall_start = time.time()
        self._config_dir = config_dir
        self._log_dir = log_dir
        self._finished = False
        self._target_threshold_m = 5
        self._target_received = False
        self._prev_imu_timestamp = 0
        self._prev_los_ned_dir = np.zeros(3)
        self._prev_pos_ned = np.zeros(3)
        self._prev_imu_ts = 0
        self._prev_ts = time.time()
        self._tracker_pos_px = None
        self._init_pos_lla_deg = None
        self._ai1Kestimates = None
        # vehicle_config_path = self._get_vehicle_config_file(self._config_dir)
        self._input_logger = Logger("input", self._log_dir, save_log_to_file=True, print_logs_to_console=False, datatype="CSV")
        # vehicle_data_parser = Config_Parser(path=vehicle_config_path)
        self.dronemass = 0.55
        # if(vehicle_data_parser is None):
        #     print("config init failed")    
        self.prev_quat_ned_desbodyfrd_cmd = None

        self.destHeight = None
        self.referencePoint = np.array([10,0,-10])
        self.tar_measurement_ned = np.array([0,0,0])
        self.heading_dir_ned = np.array([0,0,0]) 
        self.holdonHeight = None
        self.holdonHeading = None
        self.holdonPos_ned = None
        self.holdonTime = None       
        self.yawDefinedDir_ned = None   
        self.homingStage = HOMING_STAGE.NONE     

        
        ##############################
        # start scenario definitions #
        ##############################
        self.missionType = MISSION_TYPE.CIRCLE    # 1 - WAYPOINT, 2 - VELOCITY, 3 - CIRCLE, 4 - LISSAJOUS, 5 - TRACKER, 6 - SECTION
        self.yawControlType = YAW_COMMAND.HOLD_CUR_DIR   #YAW_COMMAND.CAMERA_DIR   #YAW_COMMAND.VELOCITY_DIR  # YAW_COMMAND.HOLD_CUR_DIR
        
        self.maximalVelocity = 10 # m/s (horizontal)
        self.descentVelocity = 10
        self.originOffset_frd = np.array([0,0,0])   # target waypoint in mode 1 or center of the circle in mode 3
        self.terminalHomingAlowed = True 
        self.circleRadius = 10
        
        self._control = Control(self._config_dir, self._log_dir, controller=VelocityPIDController(mass=self.dronemass), maximalVelocity=self.maximalVelocity)
        # self._control = Control(self._config_dir, self._log_dir, controller=AccelerationPIDController(mass=self.dronemass))
        # self._control = Control(self._config_dir, self._log_dir, controller=GeometricController(mass=self.dronemass))
        # self._control = Control(self._config_dir, self._log_dir, controller=AdaptiveGeometricController(mass=self.dronemass))
        # self._control = Control(self._config_dir, self._log_dir, controller=JaeyoungController(mass=self.dronemass))
        # self._control = Control(self._config_dir, self._log_dir, controller=BrescianiniController(mass=self.dronemass))
        # self._control = Control(self._config_dir, self._log_dir, controller=KooijmanController(mass=self.dronemass))
        self.rateControlEnabled = False
        
        ###############################
        # end of scenario definitions #
        ###############################
        
        topicsList = [ 
                       [zmqTopics.topicMavlinkAttitude,         zmqTopics.topicMavlinkPort],
                       [zmqTopics.topicMavlinkAttitudeRate,     zmqTopics.topicMavlinkPort],
                       [zmqTopics.topicMavlinkAltitude,         zmqTopics.topicMavlinkPort],
                       [zmqTopics.topicMavlinkHighresIMU,       zmqTopics.topicMavlinkPort],
                       [zmqTopics.topicMavlinkAttitudeQuat,     zmqTopics.topicMavlinkPort],          
                       [zmqTopics.topicMavlinkLocalPositionNed, zmqTopics.topicMavlinkPort],
                       [zmqTopics.topicMavlinkGlobalPositionInt,zmqTopics.topicMavlinkPort],
                       [zmqTopics.topicMavlinkCurrentMode,      zmqTopics.topicMavlinkPort],
                       [zmqTopics.topicMavlinkHeartbeat,        zmqTopics.topicMavlinkPort],
                       [zmqTopics.topicMavlinkVFR_HUD,          zmqTopics.topicMavlinkPort],
                       [zmqTopics.topicMavlinkDistanceSensor,   zmqTopics.topicMavlinkPort],
                       [zmqTopics.topicMavlinkOpticalFlow,      zmqTopics.topicMavlinkPort],
             ]

        self.subsSocks=[]
        self.mpsDict = {}

        for topic in topicsList:
            self.mpsDict[topic[0]] = mps.MPS(topic[0])
            self.subsSocks.append( zmqWrapper.subscribe( [ topic[0] ], topic[1] ) )
        
        self.currentData = Flight_Data()
        self.trackData = None
        self.pubSock = zmqWrapper.publisher(zmqTopics.topicGuidenceCmdPort)

        ##self._control.reset(thrust=self._hardware_adapter.get_current_thrust(), yaw=self._hardware_adapter.get_current_yaw())
        self._current_pos_lla = LLA(timestamp=0, lla=np.zeros(3))

        self.tic = time.time()
        self.dest_pos_ned = None

        # self.allInOneKalman = AllInOneKalman()

#################################################################################################################
    def _get_vehicle_config_file(self, config_dir):
        system_config_parser = Config_Parser(path=os.path.join(config_dir, "system_config.json"))
        vehicle_config_file_name =  system_config_parser.get_value("vehicle_config_file", default_value="")
        vehicle_config_path = os.path.join(self._config_dir,"vehicle", vehicle_config_file_name)
        return vehicle_config_path

#################################################################################################################
    def _log_input_data(self,   current_ts, imu_ts,
                                command, rpy_rate_cmd, quat_ned_desbodyfrd_cmd, step_dt, counter,
                                destination_ned):
        # print(target_received,trk_i, trk_j )
        self._input_logger.log({"imu_ts":self.currentData.imu_ned.timestamp,
                                "accl_ned_x": self.currentData.imu_ned.accel[0],"accl_ned_y": self.currentData.imu_ned.accel[1], "accl_ned_z": self.currentData.imu_ned.accel[2],
                                "gyro_ned_x": self.currentData.imu_ned.gyro[0], "gyro_ned_y": self.currentData.imu_ned.gyro[1], "gyro_ned_z": self.currentData.imu_ned.gyro[2],
                                "pos_ned_ts": self.currentData.pos_ned_m.timestamp,
                                "pos_ned_x": self.currentData.pos_ned_m.ned[0], "pos_ned_y": self.currentData.pos_ned_m.ned[1], "pos_ned_z": self.currentData.pos_ned_m.ned[2],
                                "vel_ned_x": self.currentData.pos_ned_m.vel_ned[0], "vel_ned_y": self.currentData.pos_ned_m.vel_ned[1], "vel_ned_z": self.currentData.pos_ned_m.vel_ned[2],
                                "quat_ned_bodyfrd_ts": self.currentData.quat_ned_bodyfrd.timestamp,
                                "quat_ned_bodyfrd_x": self.currentData.quat_ned_bodyfrd.x, "quat_ned_bodyfrd_y": self.currentData.quat_ned_bodyfrd.y,
                                "quat_ned_bodyfrd_z": self.currentData.quat_ned_bodyfrd.z, "quat_ned_bodyfrd_w": self.currentData.quat_ned_bodyfrd.w,
                                "lla_ts": self.currentData.raw_pos_lla_deg.timestamp,
                                "lla_lat_deg": self.currentData.raw_pos_lla_deg.lla[0], "lla_lon_deg": self.currentData.raw_pos_lla_deg.lla[1], "lla_alt": self.currentData.raw_pos_lla_deg.lla[2],
                                "current_ts":current_ts, "step_dt":step_dt, "imu_ts":imu_ts,
                                "current_alt": self.currentData.altitude_m.relative, "command": command, 
                                "rpy_rate_cmd_x": rpy_rate_cmd[0], "rpy_rate_cmd_y": rpy_rate_cmd[1], "rpy_rate_cmd_z": rpy_rate_cmd[2], 
                                "quat_ned_desbodyfrd_cmd_x": quat_ned_desbodyfrd_cmd.x, "quat_ned_desbodyfrd_cmd_y": quat_ned_desbodyfrd_cmd.y,
                                "quat_ned_desbodyfrd_cmd_z": quat_ned_desbodyfrd_cmd.z, "quat_ned_desbodyfrd_cmd_w": quat_ned_desbodyfrd_cmd.w,
                                "destination_ned_x": destination_ned[0], "destination_ned_y": destination_ned[1], "destination_ned_z": destination_ned[2],
                                "counter": counter})
                                 
###############################################################################################################################################
    def _get_los_ned_dir(self, track_pos_px, quat_ned_bodyfrd:Quaternion, img_size_px, camera_fov_vec, quat_bodyfrd_cam:Quaternion):
        centerPixel = img_size_px/2
        track_angle_cam = (track_pos_px - centerPixel) / img_size_px *camera_fov_vec
        
        los_dir_cam = np.array([np.sin(track_angle_cam[0]), np.sin(track_angle_cam[1]), -1])
        los_dir_cam = unitVec(los_dir_cam)
        #TODO: add here transform for camera angle as well 
        quat_ned_cam = quat_ned_bodyfrd*quat_bodyfrd_cam
        los_ned_dir = quat_ned_cam.rotate_vec(los_dir_cam)
        return los_ned_dir

###############################################################################################################################################
    def sys_manager_step(self, counter = -1, log_data=True):
        self.gatherData()

        if (not self.currentData.gathered['quat_ned_bodyfrd']) or \
            (not self.currentData.gathered['pos_ned_m']) or \
            (not self.currentData.gathered['imu_ned']) :#or \
        # (not self.currentData.gathered['tracker_px']):
            print('-return-0-')
            return
        self.holdonHeight = self.currentData.pos_ned_m.ned[2] if self.holdonHeight is None else self.holdonHeight
        self.holdonHeading = self.currentData.quat_ned_bodyfrd.rotate_vec(np.array([1, 0, 0])) if self.holdonHeading is None else self.holdonHeading
        self.holdonPos_ned = self.currentData.pos_ned_m.ned if self.holdonPos_ned is None else self.holdonPos_ned
        self.holdonTime = time.time() if self.holdonTime is None else self.holdonTime
        
        # imu_ned = deepcopy(self.currentData.imu_ned)          
        pos_ned = deepcopy(self.currentData.pos_ned_m.ned)
        
        quat_ned_bodyfrd = self.currentData.quat_ned_bodyfrd
               
        current_ts = time.time()
               
        controlType = None
        desired_trajectory = None
        headingTarget=None
        
        # self.heading_dir_ned = np.array([-15,0,0])-np.array([pos_ned[0],pos_ned[1],0]); self.heading_dir_ned=self.heading_dir_ned/np.linalg.norm(self.heading_dir_ned)
        self.heading_dir_ned = deepcopy(self.holdonHeading)
        if self.yawControlType == YAW_COMMAND.HOLD_CUR_DIR and self.yawDefinedDir_ned is not None:
            self.heading_dir_ned = deepcopy(self.yawDefinedDir_ned)    
        elif self.yawControlType == YAW_COMMAND.VELOCITY_DIR:
            if np.linalg.norm(self.currentData.pos_ned_m.vel_ned[0:2])>1:
                self.heading_dir_ned = deepcopy(self.currentData.pos_ned_m.vel_ned)
                self.heading_dir_ned[2] = 0; self.heading_dir_ned = unitVec(self.heading_dir_ned)
        elif self.yawControlType == YAW_COMMAND.CAMERA_DIR:
            self.heading_dir_ned = unitVec(np.array([self.los_ned_dir[0], self.los_ned_dir[1], 0]))
            
        missionPoint = deepcopy(self.referencePoint);   missionPoint[2] = self.holdonHeight
        if self.missionType == MISSION_TYPE.WAYPOINT:
            desired_trajectory = self.pos_point(missionPoint=missionPoint, missionAttitudeDirection=self.heading_dir_ned)
            controlType = desired_trajectory[2]   # (PosControl, VelControl, YawControl)
            self.dest_pos_ned = desired_trajectory[0][0]
            self.destHeight = desired_trajectory[0][0][2]
            self.heading_dir_ned = desired_trajectory[1][0]
         
        elif self.missionType == MISSION_TYPE.VELOCITY:
            missionVelocity = unitVec(missionPoint-self.holdonPos_ned)*self.maximalVelocity
            desired_trajectory = self.vel_point(missionVelocity=missionVelocity, missionAttitudeDirection=self.heading_dir_ned)
            controlType = desired_trajectory[2]   # (PosControl, VelControl, YawControl)
            self.dest_pos_ned = desired_trajectory[0][0]
            self.destHeight = desired_trajectory[0][0][2]
            self.heading_dir_ned = desired_trajectory[1][0]
            
        elif self.missionType == MISSION_TYPE.CIRCLE: 
            desired_trajectory = self.horz_circle(center=missionPoint, radius=self.circleRadius, Vel=self.maximalVelocity-1, missionAttitudeDirection=self.heading_dir_ned)   # bui
            # desired_trajectory = self.horz_circle(center = np.array([-10,20,0]), radius=10)    # corner ok, bui fades away
            controlType = desired_trajectory[2]   # (PosControl, VelControl, YawControl)
            self.dest_pos_ned = desired_trajectory[0][0]
            self.destHeight = desired_trajectory[0][0][2]
        
        elif self.missionType == MISSION_TYPE.LISSAJOUS:
            desired_trajectory = self.command_Lissajous()
            controlType = desired_trajectory[2]   # (PosControl, VelControl, YawControl)
            self.dest_pos_ned = desired_trajectory[0][0]
            self.destHeight = desired_trajectory[0][0][2]
            self.heading_dir_ned = desired_trajectory[1][0]
            
        elif self.missionType == MISSION_TYPE.SECTION:
            startPoint = deepcopy(self.holdonPos_ned); endPoint = deepcopy(missionPoint)
            desired_trajectory = self.lineConstVel(startPoint=startPoint, endPoint=endPoint, startTime=self.holdonTime, 
                                                speed=self.maximalVelocity, missionAttitudeDirection=self.heading_dir_ned)
            controlType = desired_trajectory[2]
            self.dest_pos_ned = desired_trajectory[0][0]
            self.destHeight = desired_trajectory[0][0][2]           
               
        trajDest_pos_ned = np.array(self.dest_pos_ned);  trajDest_pos_ned[2] = self.currentData.pos_ned_m.ned[2] if self.destHeight is None else self.destHeight
        trajDest_vel_ned = np.array([0,0,0]) if desired_trajectory is None else desired_trajectory[0][1]
        trajDest_acc_ned = np.array([0,0,0]) if desired_trajectory is None else desired_trajectory[0][2]
        
        trajDest = (trajDest_pos_ned, trajDest_vel_ned, trajDest_acc_ned)  
            
        # at HIGH_ALTITUDE_FOLLOW guidance state command is derived from trajDest
        # at TERMINAL_GUIDANCE    guidance state command is derived from los_ned_dir and los_distance
        command, rpyRate_cmd, quat_ned_desbodyfrd_cmd = self._control.get_cmd(pos_ned=self.currentData.pos_ned_m.ned, vel_ned=self.currentData.pos_ned_m.vel_ned, accel_ned=self.currentData.imu_ned.accel, 
                                                                gyro_ned=self.currentData.imu_ned.gyro,
                                                                 quat_ned_bodyfrd=quat_ned_bodyfrd,
                                                                imu_ts=self.currentData.imu_ts, step_dt=current_ts-self._prev_ts, current_ts=current_ts,                                                                
                                                                counter=counter, trajDest_ned=trajDest, controlType=controlType, 
                                                                headingDest=(self.heading_dir_ned, np.zeros(3), np.zeros(3)),
                                                                homingStage=self.homingStage)
        rollpitchlimit = np.deg2rad(20)
        rpy=quat_ned_desbodyfrd_cmd.to_euler().rpy
        rpy[0] = np.clip(rpy[0], -rollpitchlimit, rollpitchlimit)
        rpy[1] = np.clip(rpy[1], -rollpitchlimit, rollpitchlimit)
        quat_ned_desbodyfrd_cmd = Quaternion.from_euler(rpy[0], rpy[1], rpy[2])
        
        if self.prev_quat_ned_desbodyfrd_cmd is not None and self.prev_quat_ned_desbodyfrd_cmd.dot(quat_ned_desbodyfrd_cmd) < 0:
            quat_ned_desbodyfrd_cmd = -quat_ned_desbodyfrd_cmd
        
        forward_dir_frd = quat_ned_bodyfrd.rotate_vec(np.array([1,0,0]))
        yawCmd=np.arctan2(forward_dir_frd[1], forward_dir_frd[0])
        if self.yawControlType == YAW_COMMAND.NO_CONTROL:
            yawCmd = self.currentData.heading
        else:
            heading_dir_ned = self.heading_dir_ned 
            yawCmd = np.arctan2(heading_dir_ned[1], heading_dir_ned[0])
        
        if self._control.controlnode.controllerType == "VelocityPID":
            msg = { 'ts': time.time(), 'velCmd':command, 'yawCmd':yawCmd, 'yawRateCmd':0, }
            self.pubSock.send_multipart([zmqTopics.topicGuidenceCmdVelYaw, pickle.dumps(msg)])
        elif self._control.controlnode.controllerType == "AccelerationPID":
            msg = { 'ts': time.time(), 'accCmd':command, 'yawCmd':yawCmd, 'yawRateCmd':0, }
            self.pubSock.send_multipart([zmqTopics.topicGuidenceCmdAccYaw, pickle.dumps(msg)])
        else:
            msg = { 'ts': time.time(), 'thrustCmd':command, 'rpyRateCmd':rpyRate_cmd,
                'quatNedDesBodyFrdCmd':[quat_ned_desbodyfrd_cmd.w, quat_ned_desbodyfrd_cmd.x, quat_ned_desbodyfrd_cmd.y, quat_ned_desbodyfrd_cmd.z],
                'isRate':self.rateControlEnabled
            }
            self.pubSock.send_multipart([zmqTopics.topicGuidenceCmdAttitude, pickle.dumps(msg)])

        monitorTime=1
        if time.time() - self.tic >= monitorTime:
            # print('tar_pos_ned: %.3f %.3f %.3f, tar_vel_ned: %.3f %.3f %.3f'%(est_tracker_pos_ned[0], est_tracker_pos_ned[1], est_tracker_pos_ned[2], tar_vel_ned[0], tar_vel_ned[1], tar_vel_ned[2]))
            self.tic = time.time()
            deltaPos_ned = missionPoint
            if quat_ned_bodyfrd is not None:
                deltaPos_ned = missionPoint - pos_ned
                deltaPos_frd = quat_ned_bodyfrd.inv().rotate_vec(deltaPos_ned)
            print('--->referencePoint: %.3f %.3f %.3f, pos_ned:  %.3f %.3f %.3f '%( 
                self.referencePoint[0], self.referencePoint[1], self.referencePoint[2],
                pos_ned[0], pos_ned[1], pos_ned[2])+str(self.homingStage)+                
                "   Command:" + str(command)+ " deltaPos_frd:"+str(deltaPos_frd))#+"   delta_frd"+str(deltaPos_frd))
            
            try:
                print('<<--', np.linalg.norm(missionPoint[0:2]-pos_ned[0:2]))
            except:
                pass
        
        if log_data:
            self._log_input_data(current_ts=current_ts, step_dt=current_ts-self._prev_ts, imu_ts=self.currentData.imu_ts,                        
                                command=command, rpy_rate_cmd=rpyRate_cmd, quat_ned_desbodyfrd_cmd=quat_ned_desbodyfrd_cmd,
                                counter=counter, destination_ned=trajDest_pos_ned)
        send_start = time.time()
            
        # self._hardware_adapter.send_command(quat_cmd=quat_ned_desbodyfrd_cmd, rpy_rate=rpyRate_cmd, thrust=command)
        send_time = time.time()-send_start
        self._prev_imu_ts = self.currentData.imu_ts
        self._prev_ts = current_ts
        self._prev_pos_ned = self.currentData.pos_ned_m.ned
        self.prev_quat_ned_desbodyfrd_cmd = quat_ned_desbodyfrd_cmd
        
            
#################################################################################################################
    def getQuadBodyfrdCamera(self, nedYangle):
        yawCorrection = 0 #np.pi/2
        mat_cam_bodyfrd =  rotZ(-np.pi/2+yawCorrection) @ rotY(np.pi/2) @ rotY(-nedYangle) #   rotZned * rotYned * rotYned
        return Quaternion.from_matrix(mat=mat_cam_bodyfrd.T) 

#################################################################################################################
    def horz_circle(self, center=np.array([0,0,0]), radius=10, missionAttitudeDirection=None, Vel=4):
        if missionAttitudeDirection is None:
            missionAttitudeDirection = np.array([-1, 0, 0])
        t=time.time()-self._overall_start
        
        A = radius
        B = radius
        C = 0  #0.2

        R=(A+B)/2
        L=2*pi*R
        w=Vel/L

        d = pi / 2 * 0

        a = 2*pi*w  #.1
        b = 2*pi*w  #.2
        c = 2*pi*w*2  #.2

        # % t = linspace(0, 2*pi, 2*pi*100+1);
        # % x = A * sin(a * t + d);
        # % y = B * sin(b * t);
        # % z = alt + C * cos(2 * t);
        # % plot3(x, y, z);

        xd =      np.array([A *         sin(a * t + d), B *         cos(b * t), C * cos(c * t)])+center
        x_dot =  np.array([A * a *     cos(a * t + d), B * b *    -sin(b * t), C * c * -sin(c * t)])
        x_2dot =  np.array([A * a**2 * -sin(a * t + d), B * b**2 * -cos(b * t), C * c**2 * -cos(c * t)])
        x_3dot =  np.array([A * a**3 * -cos(a * t + d), B * b**3 *  sin(b * t), C * c**3 * sin(c * t)])
        x_4dot =  np.array([A * a**4 *  sin(a * t + d), B * b**4 *  cos(b * t), C * c**4 * cos(c * t)])

        b1 = missionAttitudeDirection
        b1_dot = np.array([0,0,0])  #
        b1_2dot = np.array([0,0,0])        # w = 2 * pi / 10
        # b1d = np.array([cos(w * t), sin(w * t), 0])
        # b1d_dot = w * np.array([-sin(w * t), cos(w * t), 0])
        # b1d_2dot = w**2 * np.array([-cos(w * t), -sin(w * t), 0])

        pos_control = True
        vel_control = False
        yaw_control = YAW_COMMAND.DEFINED_DIR
        return (xd, x_dot, x_2dot, x_3dot, x_4dot), (b1, b1_dot, b1_2dot), (pos_control, vel_control, yaw_control)
############################################################################################################################

    def pos_point(self, missionPoint=np.array([0, 0, -30]), missionAttitudeDirection=None):
        if missionAttitudeDirection is None:
            missionAttitudeDirection = np.array([-1, 0, 0])
        
        t=time.time()-self._overall_start
        
        x = missionPoint
        x_dot = np.array([0,0,0])
        x_2dot = np.array([0,0,0])
        x_3dot = np.array([0,0,0])
        x_4dot = np.array([0,0,0])

        b1 = missionAttitudeDirection
        b1_dot = np.array([0, 0, 0])
        b1_2dot = np.array([0, 0, 0])
        
        pos_control = True
        vel_control = False
        yaw_control = YAW_COMMAND.DEFINED_DIR
        return (x, x_dot, x_2dot, x_3dot, x_4dot), (b1, b1_dot, b1_2dot), (pos_control, vel_control, yaw_control)
################################################################################################################

    def vel_point(self, missionVelocity=np.array([1, 0, 0]), missionAttitudeDirection=None):
        if missionAttitudeDirection is None:
            missionAttitudeDirection = np.array([-1, 0, 0])
            
        t=time.time()-self._overall_start
        
        x = np.array([0, 0, 0])
        x_dot = missionVelocity
        x_2dot = np.array([0,0,0])
        x_3dot = np.array([0,0,0])
        x_4dot = np.array([0,0,0])

        b1 = missionAttitudeDirection
        b1_dot = np.array([0, 0, 0])
        b1_2dot = np.array([0, 0, 0])
        
        pos_control = False
        vel_control = True
        yaw_control = YAW_COMMAND.DEFINED_DIR
        return (x, x_dot, x_2dot, x_3dot, x_4dot), (b1, b1_dot, b1_2dot), (pos_control, vel_control, yaw_control)
################################################################################################################

    def lineConstVel(self, startPoint, endPoint, speed, startTime, missionAttitudeDirection=None):
        if missionAttitudeDirection is None:
            missionAttitudeDirection = np.array([-1, 0, 0])
            
        finished = False
        t=time.time()-startTime
        
        deltaDistance = endPoint-startPoint
        deltaDir = unitVec(deltaDistance)
        deltaDistance = np.linalg.norm(deltaDistance)
        velocity = deltaDir * speed
        x = startPoint + t*velocity
        x_dot = velocity
        if t*speed > deltaDistance:
            x = endPoint
            x_dot = np.array([0,0,0])
            finished = True
        x_2dot = np.array([0,0,0])
        x_3dot = np.array([0,0,0])
        x_4dot = np.array([0,0,0])

        b1 = missionAttitudeDirection
        b1_dot = np.array([0, 0, 0])
        b1_2dot = np.array([0, 0, 0])
        
        pos_control = True
        vel_control = t*speed < deltaDistance
        yaw_control = YAW_COMMAND.DEFINED_DIR
        return (x, x_dot, x_2dot, x_3dot, x_4dot), (b1, b1_dot, b1_2dot), (pos_control, vel_control, yaw_control), finished
##############################################################################################################
    def vert_circle(self):

        t=time.time()-self._overall_start
        
        Vel=3

        A = 5
        B = 5
        C = 0  #0.2

        R=(A+B)/2
        L=2*pi*R
        w=Vel/L

        d = pi / 2 * 0

        a = 2*pi*w  #.1
        b = 2*pi*w  #.2
        c = 2*pi*w*2  #.2
        alt = -20

        # % t = linspace(0, 2*pi, 2*pi*100+1);
        # % x = A * sin(a * t + d);
        # % y = B * sin(b * t);
        # % z = alt + C * cos(2 * t);
        # % plot3(x, y, z);

        x =      np.array([C * cos(c * t),         A *         sin(a * t + d), B *         cos(b * t), alt + C * cos(c * t)])
        x_dot =  np.array([C * c * -sin(c * t),    A * a *     cos(a * t + d), B * b *    -sin(b * t)])
        x_2dot =  np.array([C * c**2 * -cos(c * t), A * a**2 * -sin(a * t + d), B * b**2 * -cos(b * t)])
        x_3dot =  np.array([C * c**3 *  sin(c * t), A * a**3 * -cos(a * t + d), B * b**3 *  sin(b * t)])
        x_4dot =  np.array([C * c**4 *  cos(c * t), A * a**4 *  sin(a * t + d), B * b**4 *  cos(b * t)])

        b1 = np.array([1,0,0])
        b1_dot = np.array([0,0,0])  #
        b1_2dot = np.array([0,0,0])        # w = 2 * pi / 10
        # self.b1d = np.array([cos(w * t), sin(w * t), 0])
        # self.b1d_dot = w * np.array([-sin(w * t), cos(w * t), 0])
        # self.b1d_2dot = w**2 * np.array([-cos(w * t), -sin(w * t), 0])

        pos_control = False
        vel_control = True
        yaw_control = YAW_COMMAND.DEFINED_DIR
        return (x, x_dot, x_2dot, x_3dot, x_4dot), (b1, b1_dot, b1_2dot), (pos_control, vel_control, yaw_control)
###############################################################################################################
    
    def command_Lissajous(self, t=None):
        if t is None:
            t=time.time()-self._overall_start
        
        # Lissajous curve
        # x=A*sin(a*t+d), y=B*sin(b*t), z=alt+C*cos(c*t)
        # several common parameters:
        # a:b   d    0       pi/4      pi/2      3/4*pi    pi
        # 1:1        /      ellipse   circle    ellipce    \
        # 1:2        )      8-figure  8-figure  8-figure   (
        # 1:3        S      
        
        A = 15   # X amplitude
        B = 15   # Y amplitude
        C = 5 # Z amplitude
        alt = -1  # Z offset

        d = pi / 2 * 0

        a = .2*1.5  # X frequency
        b = .3*1.5  # Y frequency
        c = .2    # Z frequency
        w = 2 * pi / 10  # attitude rotation frequency
        (x, x_dot, x_2dot, x_3dot, x_4dot), (b1, b1_dot, b1_2dot) = lissajous_func(t, A=A, B=B, C=C, a=a, b=b, c=c, alt=alt, w = w)
        
        pos_control = False
        vel_control = True
        yaw_control = YAW_COMMAND.DEFINED_DIR
        return (x, x_dot, x_2dot, x_3dot, x_4dot), (b1, b1_dot, b1_2dot), (pos_control, vel_control, yaw_control)
#################################################################################################################
    def gatherData(self):
        socks = zmq.select(self.subsSocks, [], [], timeout=0.001)
        if len(socks)>0:
            for sock in socks:
                if len(sock) == 0:
                    continue
                ret = sock[0].recv_multipart()
                #topic,data=ret[0],pickle.loads(ret[1])
                topic = ret[0]
                if topic in self.mpsDict.keys():
                    self.mpsDict[topic].calcMPS()
                data = pickle.loads(ret[1])
                
                if topic == zmqTopics.topicMavlinkAttitude:                           # mavlink ATTITUDE
                    time_boot_ms = data['time_boot_ms']        # Flight controller time
                    ts = data['local-ts']
                    roll       = np.deg2rad(data['roll'])
                    pitch      = np.deg2rad(data['pitch'])
                    yaw        = np.deg2rad(data['yaw'])
                    self.currentData.rpy2 = np.array([roll, pitch, yaw])
                    self.currentData.gathered['euler_ned_bodyfrd']=True
                    
                elif topic == zmqTopics.topicMavlinkAttitudeRate:                           # mavlink ATTITUDE
                    time_boot_ms = data['time_boot_ms']        # Flight controller time
                    ts = data['local-ts']
                    rollspeed  = np.deg2rad(data['rollspeed'])
                    pitchspeed = np.deg2rad(data['pitchspeed'])
                    yawspeed   = np.deg2rad(data['yawspeed'])
                    self.currentData.rpy2_rates = np.array([rollspeed, pitchspeed, yawspeed])
                    
                elif topic == zmqTopics.topicMavlinkHighresIMU:                              # mavlink HIGHRES_IMU
                # Flight controller time
                    self.currentData.imu_ned.gyro           = np.array([data['xgyro'], data['ygyro'], data['zgyro']  ])
                    self.currentData.imu_ned.accel          = np.array([data['xacc'], data['yacc'], data['zacc']  ])
                    self.currentData.altitude_m.relative    = data['pressure_alt']
                    self.currentData.imu_ned.timestamp = data['local-ts']
                    self.currentData.imu_ts = data['time_usec']/1e6 
                    self.currentData.gathered['imu_ned']=True                    
                    
                    #if (self.currentData.custom_mode_id == 50593792) or \ # and self.currentData.groundspeed<0.5) or  \
                    if self.currentData.custom_mode_id == 50593792 or \
                       self.currentData.custom_mode_id == 196608  or\
                       self.currentData.custom_mode_id == 131072 or \
                       self.currentData.custom_mode_id == 65535 or \
                       self.currentData.custom_mode_id ==84148224:  # HOLD ON state or POSITION state
                        
                        self.holdonHeight = self.currentData.pos_ned_m.ned[2]
                        self.yawDefinedDir_ned = np.array([np.cos(self.currentData.heading), np.sin(self.currentData.heading), 0])
                        self.holdonHeading = self.yawDefinedDir_ned
                        self.holdonPos_ned = self.currentData.pos_ned_m.ned
                        self.holdonTime = time.time()
                        
                        if self.currentData.throttle>5:
                            self._control.MaximalThrust = self._control.controlnode.param.mass*9.81/self.currentData.throttle*100
                            self._control.controlnode.resetIntegralErrorTerms()
                        
                        self.homingStage = HOMING_STAGE.SECTION if self.missionType == MISSION_TYPE.TRACKER else HOMING_STAGE.NONE
                        self.currentData.offboardMode = False
                        
                    elif self.currentData.custom_mode_id == 393216:  # OFFBOARD state
                        self.currentData.offboardMode = True
                        if not self._control.controlnode.use_integralTerm:
                            self._control.controlnode.resetIntegralErrorTerms()
                            self._control.controlnode.use_integralTerm = True
                            
                elif topic == zmqTopics.topicMavlinkAttitudeQuat:                         # mavlink ATTITUDE_QUATERNION
                    time_boot_ms = data['time_boot_ms']        # Flight controller time
                    ts = data['local-ts']                            # time of receiving the message
                    w = data['q1']
                    x = data['q2']
                    y = data['q3']
                    z = data['q4']

                    self.currentData.quat_ned_bodyfrd.set(x=x, y=y,z=z,w=w,timestamp=ts) 
                    self.currentData.rpy_rates = np.array([rollrate, pitchrate, yawrate])
                    self.currentData.gathered['quat_ned_bodyfrd']=True

                elif topic == zmqTopics.topicMavlinkLocalPositionNed:               # mavlink LOCAL_POSITION_NED
                    time_boot_ms = data['time_boot_ms']        # Flight controller time
                    self.currentData.pos_ned_m.ned       = np.array([data['x'], data['y'], data['z']])
                    self.currentData.pos_ned_m.vel_ned   = np.array([data['vx'], data['vy'], data['vz']])
                    self.currentData.pos_ned_m.timestamp = data['local-ts']
                    self.currentData.gathered['pos_ned_m']=True
                    
                elif topic == zmqTopics.topicMavlinkGlobalPositionInt:                    # navlink GLOBAL_POSITION_INT
                    time_boot_ms = data['time_boot_ms']        # Flight controller time
                    ts = data['local-ts']                            # time of receiving the message
                    lat = data['lat']   # deg
                    lon = data['lon']   # deg
                    alt = data['alt']   # m
                    relative_alt = data['relative_alt']
                    self.currentData.filt_pos_lla_deg = LLA(timestamp=ts, lla=[lat, lon, alt])
                    
                elif topic == zmqTopics.topicMavlinkDistanceSensor:   
                    pass
                    
                elif topic == zmqTopics.topicMavlinkCurrentMode:                    # mavlink CURRENT_MODE
                    ts = data['ts']                            # time of receiving the message
                    standart_mode = data['standart_mode']
                    custom_mode   = data['custom_mode']
                    intended_custom_mode = data['intended_custom_mode']
                    
                elif topic == zmqTopics.topicMavlinkHeartbeat:                    # mavlink CURRENT_MODE
                    ts = data['local-ts']                            # time of receiving the message
                    type = data['type']
                    autopilot = data['autopilot']
                    base_mode   = data['base_mode']
                    self.currentData.custom_mode_id = data['custom_mode']
                    system_status = data['system_status']
                    mavlink_version = data['mavlink_version']
                    mode_string = data['mode_string']

                elif topic == zmqTopics.topicMavlinkVFR_HUD:                    # mavlink VFR_HUD
                    ts = data['local-ts']                            # time of receiving the message
                    airspeed = data['airspeed']
                    groundspeed = data['groundspeed']
                    heading_deg = data['heading']       # heding in degrees from north
                    self.currentData.throttle = data['throttle']
                    alt = data['alt']
                    climb = data['climb']
                    self.currentData.heading = np.deg2rad(heading_deg)

            
############################################################################################################################
############################################################################################################################
############################################################################################################################
############################################################################################################################
############################################################################################################################
if __name__=='__main__':
    sysMgr = System_Manager(log_dir='../logs/', config_dir='config/')
    while True:
        time.sleep(0.0001)
        sysMgr.sys_manager_step()

