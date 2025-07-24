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

sockPub = zmqWrapper.publisher(zmqTopics.topicMavlinkPort)

subSock = zmqWrapper.subscribe([zmqTopics.topicGuidenceCmdAttitude, 
                                zmqTopics.topicGuidenceCmdNedAttitude,
                                zmqTopics.topicGuidenceCmdVelAttitude,
                                zmqTopics.topicGuidenceCmdAccAttitude,
                                ], zmqTopics.topicGuidenceCmdPort)

mavlinkMpsCnt = 0
mavlinkMpsTic = time.time()

quatCnt = 0

@drone.on_message('*')
def listener(self, name, message):  
    global mavlinkMpsCnt
    m=message.to_dict()
    m['ts_receive']=time.time()
    ## optional: save all mavlink messages - HERE
    processMavlink(m)
    mavlinkMpsCnt += 1

gpsMpsCnt = 0
gpsMpsTic = time.time()

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

        name=m['mavpackettype']
        ts = time.time()
        msg = None
        if 'ATTITUDE' == name:
            msg = {'topic':zmqTopics.topicMavlinkAttitude}
            msg['ts']        = ts
            msg['time_boot_ms'] = m['time_boot_ms']
            msg['yaw']       = m['yaw']
            msg['pitch'] = m['pitch']
            msg['roll']  = m['roll']
            msg['rollspeed']  = m['rollspeed']
            msg['pitchspeed']  = m['pitchspeed']
            msg['yawspeed']  = m['yawspeed']

        elif 'DISTANCE_SENSOR' == name:
            msg = {'topic':zmqTopics.topicMavlinkDistanceSensor}
            msg['ts'] = ts
            msg['current_distance'] = m['current_distance']
            
        elif 'HIGHRES_IMU' == name:
            msg            = {'topic':zmqTopics.topicMavlinkIMU}
            msg['ts']      = ts
            msg['time_usec'] = m['time_usec']
            msg['pressure_alt'] = m['pressure_alt']
            msg['xacc'] = m['xacc']
            msg['yacc'] = m['yacc']
            msg['zacc'] = m['zacc']
            msg['xgyro'] = m['xgyro']
            msg['ygyro'] = m['ygyro']
            msg['zgyro'] = m['zgyro']
            msg['state'] = str(drone.mode)

        elif 'LOCAL_POSITION_NED' == name:
            msg            = {'topic':zmqTopics.topicMavlinkLocalPositionNed}
            msg['ts']      = ts
            msg['time_boot_ms'] = m['time_boot_ms']
            msg['pos_ned_m'] = {'ned':[0]*3}
            msg['pos_ned_m']['ned'][0] = m['x']
            msg['pos_ned_m']['ned'][1] = m['y']
            msg['pos_ned_m']['ned'][2] = m['z']
            msg['pos_ned_m']['vel_ned'] = [0]*3
            msg['pos_ned_m']['vel_ned'][0] = m['vx']
            msg['pos_ned_m']['vel_ned'][1] = m['vy']
            msg['pos_ned_m']['vel_ned'][2] = m['vz']
            '''
            msg = {}
            msg['ts'] = ts
            msg['EKF_ALT']=-m['z']
            
            gs.ekf_ne = (m['x'],m['y'])
            gs.gnd_speed = np.sqrt(m['vx']**2+m['vy']**2)
            gs.ekf_vne = (m['vx'],m['vy'])
            gs.vz=m['vz']
            if last_position_ned is not None:
                dt=(m['time_boot_ms']-last_position_ned['time_boot_ms'])/1000.0
                gs.accz = (m['vz']-last_position_ned['vz'])/dt
            gs.mission_status['GND_SPD']='{:03.1f}'.format(gs.gnd_speed)
            gs.mission_status['V_SPD']='{:03.1f}'.format(m['vz'])
            if gs.fd_debug is not None:
                pickle.dump(('mavlink_ned',time.time(),m),gs.fd_debug)
            last_position_ned=m
            '''

        elif 'VIBRATION' == name:
            pass
            
        elif 'GPS_RAW_INT' == name:
            gpsMpsCnt += 1
            
            msg = {'topic':zmqTopics.topicMavlinkRawGPS}
            msg['ts']      = ts
            msg['time_usec'] = m['time_usec']
            msg['GPS_FT']  = m['fix_type']
            msg['GPS_SAT'] = m['satellites_visible']
            msg['GPS_VEL'] = m['vel']/100.0 # [cm/m]
            #tosend = (m['lat']*1e-7,m['lon']*1e-7,m['alt']/100,np.degrees(m['yaw']))
            #chck if already converted (bug...)
            if int(m['fix_type'])>0:
                msg['lat'] = m['lat']*1e-7
                msg['lon'] = m['lon']*1e-7
                msg['alt'] = m['alt']/1000 #[m]
        
        elif 'GLOBAL_POSITION_INT' == name:
            msg = {'topic': zmqTopics.topicMavlinkFilteredGPS}
            msg['ts'] = ts
            msg['time_boot_ms'] = m['time_boot_ms']
            msg['lat'] = m['lat'] * 1e-7
            msg['lon'] = m['lon'] * 1e-7
            msg['alt'] = m['alt'] / 1000  # [m]
            msg['relative_alt'] = m['relative_alt'] / 1000  # [m]
            msg['vx'] = m['vx'] / 100.0
            msg['vy'] = m['vy'] / 100.0
            msg['vz'] = m['vz'] / 100.0
            msg['hdg'] = m['hdg'] / 100.0

        elif 'VFR_HUD'==name:
            msg = {'topic': zmqTopics.topicMavlinkVFR_HUD}
            msg['ts']       = ts
            msg['airspeed'] = float(m['airspeed'])
            msg['groundspeed'] = float(m['groundspeed'])
            msg['heading'] = (m['heading'])
            msg['throttle'] = (m['throttle'])
            msg['alt'] = float(m['alt'])
            msg['climb'] = float(m['climb'])

        elif name =='ATTITUDE_QUATERNION':  # 50 HZ
            quatCnt += 1
            msg = {'topic': zmqTopics.topicMavlinkAttQuat}

            msg['ts']                   = ts
            msg['time_boot_ms']   = int(m['time_boot_ms'])
            msg['w']                    = float(m['q1'])
            msg['x']                    = float(m['q2'])
            msg['y']                    = float(m['q3'])
            msg['z']                    = float(m['q4'])
            msg['rollspeed']            = float(m['rollspeed'])
            msg['pitchspeed']           = float(m['pitchspeed'])
            msg['yawspeed']             = float(m['yawspeed'])
        
        elif name=='CURRENT_MODE':
            print('current mode+++++++++++++++++++++++++++++')
            msg = {'topic': zmqTopics.topicMavlinkCurrentMode}
            msg['ts'] = ts
            msg['standart_mode'] = m['standart_mode']
            msg['custom_mode'] = m['custom_mode']
            msg['intended_custom_mode'] = m['intended_custom_mode']
                       
        elif 'HEARTBEAT' == name:
            pass
            msg = {'topic': zmqTopics.topicMavlinkHeartbeat}
            msg['ts'] = ts
            msg['type'] = m['type']
            msg['autopilot'] = m['autopilot']
            msg['base_mode'] = m['base_mode']
            msg['custom_mode'] = m['custom_mode']
            msg['system_status'] = m['system_status']
            msg['mavlink_version'] = m['mavlink_version']
            
        elif name=='PARAM_VALUE' or \
             name=='ALTITUDE' or \
             name=='ATTITUDE_TARGET' or \
             name=='POSITION_TARGET_LOCAL_NED' or \
             name=='TIMESYNC' or \
             name=='ACTUATOR_CONTROL_TARGET' or \
             name=='EXTENDED_SYS_STATE' or \
             name=='SERVO_OUTPUT_RAW' or\
             name=='SYS_STATUS' or \
             name=='HOME_POSITION' or \
             name=='COMMAND_LONG' or \
             name=='PING' or \
             name=='BATTERY_STATUS' or \
             name=='SYSTEM_TIME' or \
             name=='AUTOPILOT_VERSION' or \
             name=='COMMAND_ACK' or \
             name=='ESTIMATOR_STATUS' or \
             name=='POSITION_TARGET_GLOBAL_INT':
            pass
        
        elif name=='STATUSTEXT':
            # msg['ts'] = ts
            # msg['text'] = m['text']
            # msg['severity'] = m['severity']
            # msg['ts_receive'] = m['ts_receive']
            print('STATUSTEXT: %s'%m['text'])
            
        else:
            #print('unknown message: %s'%m['mavpackettype'])
            return

        if msg is not None:
            sockPub.send_multipart([msg['topic'], pickle.dumps(msg)])
                
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
            elif topic == zmqTopics.topicGuidenceCmdNedAttitude:
                
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
                #print(msg)
                mavMsg = drone.message_factory.set_position_target_local_ned_encode(
                        0,       # time_boot_ms (not used)
                        0, 0,    # target system, target component
                        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
                        0b000111111000 , # #pos based
                        msg['nedDesBodyFrdCmd']['north'], 
                        msg['nedDesBodyFrdCmd']['east'], 
                        msg['nedDesBodyFrdCmd']['down'], # x, y, z positions (or North, East, Down in the MAV_FRAME_BODY_NED frame
                        msg['nedDesBodyFrdCmd']['vx'], 
                        msg['nedDesBodyFrdCmd']['vy'], 
                        msg['nedDesBodyFrdCmd']['vz'], # x, y, z velocity in m/s
                        0,  # x acceleration (not supported yet, ignored in GCS_Mavlink)
                        0,  # y acceleration (not supported yet, ignored in GCS_Mavlink)
                        0,  # z acceleration (not supported yet, ignored in GCS_Mavlink)
                        msg['nedDesBodyFrdCmd']['targetYaw'],
                        msg['nedDesBodyFrdCmd']['yawRate'])    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
            elif topic == zmqTopics.topicGuidenceCmdVelAttitude:
                
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
            elif topic == zmqTopics.topicGuidenceCmdAccAttitude:
                
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
