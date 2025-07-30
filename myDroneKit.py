import zmqTopics
import zmqWrapper

import asyncio
import time
import pickle
import zmq

from dronekit import connect, mavutil, Vehicle
from dronekit.lib import VehicleMode
#import mavutilsFix
#mavutil.interpert_px4_mode=mavutilsFix.interpret_px4_mode
#from pymavlink import mavutil


import pymavlink

import numpy as np
import argparse


parser = argparse.ArgumentParser(description='myDronekit...', formatter_class=argparse.RawTextHelpFormatter)
parser.add_argument('-s', '--isSim', action='store_true', help='set sim mode')
args = parser.parse_args()

isSim = True  #args.isSim

if isSim:
    drone   = connect('udp:127.0.0.1:14540') ## for Sim
else:
    drone   = connect('udp:127.0.0.1:14551')

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

sockPub = zmqWrapper.publisher(zmqTopics.topicMavlinkPort)

subSock = zmqWrapper.subscribe([zmqTopics.topicGuidenceCmdAttitude, 
                                zmqTopics.topicGuidenceCmdVelYaw,
                                zmqTopics.topicGuidenceCmdAccYaw,
                                zmqTopics.topicGuidenceCmdTakeoff,
                                zmqTopics.topicGuidenceCmdLand,
                                zmqTopics.topicGuidanceCmdArm,
                                ], zmqTopics.topicGuidenceCmdPort)

mavlinkMpsCnt = 0
mavlinkMpsTic = time.time()

quatCnt = 0

@drone.on_message('*')
def listener(self, name, message):  
    global mavlinkMpsCnt
    m=message.to_dict()
    m['local-ts']=time.time()
    if(m['mavpackettype'] == 'HEARTBEAT'):
        m['mode_string'] = ( mavutil.mode_string_v10(message))
    ## optional: save all mavlink messages - HERE
    processMavlink(m)
    mavlinkMpsCnt += 1

gpsMpsCnt = 0
gpsMpsTic = time.time()

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

def processMavlink(m):
    global gpsMpsCnt, gpsMpsTic, quatCnt
    try:

        if time.time()-gpsMpsTic >= 3:
            gpsMps  = gpsMpsCnt/(time.time()-gpsMpsTic)
            quatMps = quatCnt/(time.time()-gpsMpsTic)
            print('gps msp: %0.2f, quaternion MPS: %0.2f'%(gpsMps, quatMps))
            gpsMpsCnt = 0
            quatCnt   = 0
            gpsMpsTic = time.time()

        if m['mavpackettype'] in RELEVANT_MAVLINK_MESSAGES:
            ind = RELEVANT_MAVLINK_MESSAGES.index(m['mavpackettype'])
            sockPub.send_multipart([pubTopicsList[ind][0], pickle.dumps(m)])       
            
        elif m['mavpackettype']=='PARAM_VALUE' or \
             m['mavpackettype']=='ALTITUDE' or \
             m['mavpackettype']=='ATTITUDE_TARGET' or \
             m['mavpackettype']=='POSITION_TARGET_LOCAL_NED' or \
             m['mavpackettype']=='TIMESYNC' or \
             m['mavpackettype']=='ACTUATOR_CONTROL_TARGET' or \
             m['mavpackettype']=='EXTENDED_SYS_STATE' or \
             m['mavpackettype']=='SERVO_OUTPUT_RAW' or\
             m['mavpackettype']=='SYS_STATUS' or \
             m['mavpackettype']=='HOME_POSITION' or \
             m['mavpackettype']=='COMMAND_LONG' or \
             m['mavpackettype']=='PING' or \
             m['mavpackettype']=='BATTERY_STATUS' or \
             m['mavpackettype']=='SYSTEM_TIME' or \
             m['mavpackettype']=='AUTOPILOT_VERSION' or \
             m['mavpackettype']=='COMMAND_ACK' or \
             m['mavpackettype']=='ESTIMATOR_STATUS' or \
             m['mavpackettype']=='POSITION_TARGET_GLOBAL_INT':
            pass
        
        elif m['mavpackettype']=='STATUSTEXT':
            # msg['ts'] = ts
            # msg['text'] = m['text']
            # msg['severity'] = m['severity']
            # msg['ts_receive'] = m['ts_receive']
            print('STATUSTEXT: %s'%m['text'])
            
        else:
            #print('unknown message: %s'%m['mavpackettype'])
            return
                
    except Exception as E:
        import traceback
        traceback.print_exc()
        

async def monitorMavlink():
    global mavlinkMpsCnt, mavlinkMpsTic
        
    while True:
        time.sleep(0.0001)
        if time.time() - mavlinkMpsTic >= 5:
            mavlinkMps = mavlinkMpsCnt/(time.time() - mavlinkMpsTic)
            print('mavlink total mps: %0.2f'%mavlinkMps)
            mavlinkMpsCnt = 0
            mavlinkMpsTic = time.time()
        
        ret = zmq.select([subSock], [], [], 0.001)[0]
        if len(ret) > 0:
            data  = subSock.recv_multipart()
            topic = data[0]
            mavMsg = None
            if topic == zmqTopics.topicGuidenceCmdAttitude:
                ## auto offboard
                #if drone.mode != VehicleMode("OFFBOARD"):
                #    drone.mode = VehicleMode("OFFBOARD")
               
                msg = pickle.loads(data[1])
                if msg['isRate']:
                    typeMask = (pymavlink.mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE)
                else:
                    typeMask = (pymavlink.mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE |
                                pymavlink.mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE |
                                pymavlink.mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE)

                '''
                ## this option also works the same
                mavMsg = drone.message_factory.set_attitude_target_encode(
                0,
                0, 0,
                typeMask,
                msg['quatNedDesBodyFrdCmd'],
                msg['rpyRateCmd'][0],
                msg['rpyRateCmd'][1],
                msg['rpyRateCmd'][2],
                msg['thrustCmd']
                
                )

                print('--->', msg)

                drone.send_mavlink(mavMsg)

                '''
                
                #print('--->', msg)
                drone._master.mav.set_attitude_target_send(
                    0, #time_ms
                    0,
                    1,
                    typeMask,
                    #0b00000111,
                    msg['quatNedDesBodyFrdCmd'],
                    msg['rpyRateCmd'][0],
                    msg['rpyRateCmd'][1],
                    msg['rpyRateCmd'][2],
                    msg['thrustCmd']
                )
            elif topic == zmqTopics.topicGuidenceCmdVelYaw:
                           
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
                msg = pickle.loads(data[1])
                mavMsg = drone.message_factory.set_position_target_local_ned_encode(
                        0,       # time_boot_ms (not used)
                        0, 0,    # target system, target component
                        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame                    
                        0b100111000111 , # Vel based + Yaw
                        0, 
                        0,
                        0, 
                        msg['velCmd'][0], 
                        msg['velCmd'][1], 
                        msg['velCmd'][2], # x, y, z velocity in m/s
                        0,  
                        0,  
                        0,  
                        msg['yawCmd'],
                        0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
            elif topic == zmqTopics.topicGuidenceCmdAccYaw:
                
                msg = pickle.loads(data[1])  
                mavMsg = drone.message_factory.set_position_target_local_ned_encode(
                        0,       # time_boot_ms (not used)
                        0, 0,    # target system, target component
                        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
                        0b100000111111 , # #Acceleration based
                        0,  
                        0, 
                        0, 
                        0, 
                        0, 
                        0, # x, y, z velocity in m/s
                        msg['accCmd'][0],  # x acceleration (not supported yet, ignored in GCS_Mavlink)
                        msg['accCmd'][1],  # y acceleration (not supported yet, ignored in GCS_Mavlink)
                        msg['accCmd'][2],  # z acceleration (not supported yet, ignored in GCS_Mavlink)
                        msg['yawCmd'],
                        0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
                #print('---',np.degrees(target_yaw),np.degrees(yaw))
            if mavMsg is not None:
                drone.send_mavlink(mavMsg)
                drone.flush()
                
                
                
                
                


async def main():
    procs = [
                monitorMavlink(),
            ]

    await asyncio.gather(
                *procs
            )

if __name__=='__main__':
    loop = asyncio.get_event_loop()
    result = loop.run_until_complete(main())
