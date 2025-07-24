#!/bin/python3
from common import *
from MathUtils import Integrate, advanceRK4
import numpy as np
from numpy import sin, cos, tan, arctan2, sqrt, pi, log10, abs, log
from enum import Enum
from matplotlib import pyplot as plt

# accel = N * V_closing * lambda_dot
class GUIDANCE_TYPE(Enum):
    TRUE_PN = 0
    PURE_PN = 1
    AUGEMENTED_PN = 2
    PURE_PURSUIT = 3
    ZEM = 4
"""
TODO: maybe if lambda dot is very low add in a factor of angular error
TODO: maybe if target is close to edge of fov increase factor of angular error
LPF: data = f(n-1)*a + (1-a)*f(n)
"""
###########################################################################################################################################
###########################################################################################################################################
###########################################################################################################################################
class Guidance():
    def __init__(self, N, type:GUIDANCE_TYPE):
        self.N = N
        fc = 3
        fs = 10
        tau = 1/(2*np.pi*fc)
        T = 1/fs
        alpha = tau/(tau+T)
        # alpha = 0.2
        # print("alpha", alpha) #0.34

        # self._low_pass_filter = Low_Pass_Filter(alpha=np.radians(100), is_angle=True, type=LPF_TYPE.OTHER)
        self.K = 5
        self._type = type
        self._max_jump = 0.1
        self._prev_accel_cmd = None
        self._lambda_dot = 0
###########################################################################################################################################
    def guidance_step(self, vrel_ned, los_ned_dir, step_dt, prev_los_ned_dir = None, distance_to_target=None, vclose=None):        
        accel_cmd_ned = np.zeros(3)
        if(prev_los_ned_dir is None):
            prev_los_ned_dir = los_ned_dir
            # return 0,0
        
        if(self._type == GUIDANCE_TYPE.PURE_PURSUIT):
            # if np.dot(vrel_ned, los_ned_dir) < 0:
            #     accel_cmd_dir = los_ned_dir
            #     accel_mag = 1
            # else:
            accel_cmd_dir = np.cross(np.cross(vrel_ned, los_ned_dir), vrel_ned); 

            if(np.linalg.norm(accel_cmd_dir) > 0):
                accel_cmd_dir = accel_cmd_dir/np.linalg.norm(accel_cmd_dir)
            accel_mag = np.linalg.norm(vrel_ned - np.dot(vrel_ned, los_ned_dir)*los_ned_dir)

            accel_cmd_ned = self.N * accel_cmd_dir * accel_mag
        elif (self._type == GUIDANCE_TYPE.ZEM and distance_to_target is not None):
            # ZEM guidance
            t_go = distance_to_target / np.abs(np.dot(vrel_ned, los_ned_dir))
            los_ned = distance_to_target * los_ned_dir
            zem_ned = los_ned + vrel_ned * t_go
            zem_ned_los = los_ned_dir * np.dot(zem_ned , los_ned_dir)
            zem_ned_norm = zem_ned - zem_ned_los

            accel_cmd_ned = self.N * zem_ned_norm/(t_go*t_go)
            
        else:
            cosTheta = np.dot(los_ned_dir, prev_los_ned_dir)
            sinTheta = np.linalg.norm(np.cross(los_ned_dir, prev_los_ned_dir))
            theta = np.arctan2(sinTheta, cosTheta) # theta is the angle between current los and previous los
            theta_dot = abs(theta/step_dt)
            lamdadot_ned_dir = np.cross(prev_los_ned_dir, los_ned_dir); 
            
            if(np.linalg.norm(lamdadot_ned_dir) > 0):
                lamdadot_ned_dir = lamdadot_ned_dir/np.linalg.norm(lamdadot_ned_dir)
            else:
                lamdadot_ned_dir = np.zeros(3)
            lamdadot_ned = lamdadot_ned_dir * theta_dot
                        
            if(self._type == GUIDANCE_TYPE.TRUE_PN):
                accel_cmd_ned = self.N * np.cross(lamdadot_ned, los_ned_dir)*np.linalg.norm(vrel_ned)

            elif(self._type == GUIDANCE_TYPE.PURE_PN):
                accel_cmd_ned = self.N * np.cross(vrel_ned, lamdadot_ned)
                
        vel_des_ned = vrel_ned + accel_cmd_ned*step_dt
        
        if vclose is not None and np.linalg.norm(vrel_ned) < np.linalg.norm(vclose)*0.7:         
            accel_cmd_ned += unitVec(vclose)*(vclose-vrel_ned)/1  # acceleration to vclose in one second
            vel_des_ned += (vclose - vrel_ned)
            
        return accel_cmd_ned, vel_des_ned
####################################################################################################################################
####################################################################################################################################
######################  GUIDANCE TEST  #############################################################################################
####################################################################################################################################
####################################################################################################################################
def calculateTimeDerivative(t, y, dt, yprev, tar_pos, tar_vel, tar_accel=None, mis_accel_enu=None, calcGuidance=True):
    mis_pos = y[0:3]
    mis_vel = y[3:6]
    
    tar_accel_ned = np.zeros(3)            
    if tar_accel is not None:
        tar_accel_dir = np.cross(tar_vel, [0, 0, 1])
        tar_accel_dir = tar_accel_dir/np.linalg.norm(tar_accel_dir)
        tar_accel_ned = tar_accel_dir*tar_accel

    los = tar_pos - mis_pos
    los_dir = los/np.linalg.norm(los)
    vrel = tar_vel - mis_vel
    if yprev is None:
        prev_los_dir = los_dir
        prev_ts = t-0.01
        accel_cur = np.zeros(3)
    else:
        prev_mis_pos = yprev[0:3]
        prev_mis_vel = yprev[3:6]
        prev_tar_pos = tar_pos-(tar_vel-tar_accel_ned*dt)*dt-tar_accel_ned*dt*dt/2
        prev_los = prev_tar_pos - prev_mis_pos
        prev_los_dir = prev_los/np.linalg.norm(prev_los)
        accel_cur = (mis_vel-prev_mis_vel)/dt
        
    
    mis_accel_cmd = np.zeros(3)
    if calcGuidance:         
        if t>3:
            t=t+0                
        mis_accel_cmd,vel_cmd_ned = guidance.guidance_step(vrel_ned = vrel, los_ned_dir = los_dir, step_dt = dt, 
                                                           prev_los_ned_dir = prev_los_dir, distance_to_target=np.linalg.norm(los))
        
    if mis_accel_enu is not None:
        mis_accel_cmd = mis_accel_cmd + mis_accel_enu
            
    tau = 0.5
    MAX_ACCEL = 25
    alpha = tau/(tau+dt)
    mis_veldot = accel_cur*alpha+mis_accel_cmd*(1-alpha)
    accl_cmd=np.linalg.norm(mis_veldot)
    accl_cmd_clipped = np.clip(accl_cmd, 0, MAX_ACCEL)
    mis_veldot = mis_veldot/accl_cmd*accl_cmd_clipped
    
    mis_posdot = mis_vel
    return np.concatenate((mis_posdot, mis_veldot))

###########################################################################################################################################
def eventFunc(t, y, dt, yprev, tar_pos):
    mis_pos = y[0:3]
    if yprev is None:
        mis_vel = y[3:6]
        mis_pos_prev = mis_pos - mis_vel*dt
    else:
        mis_pos_prev = yprev[0:3]

    mis_vec_prev2cur = mis_pos - mis_pos_prev
    mis_vec_prev2cur_dir = mis_vec_prev2cur/np.linalg.norm(mis_vec_prev2cur)
    vec_prev2tar = tar_pos - mis_pos_prev
    closest_point = mis_pos_prev + np.dot(vec_prev2tar, mis_vec_prev2cur_dir)*mis_vec_prev2cur_dir
    dist_to_target = np.linalg.norm(tar_pos - closest_point)
    return (np.linalg.norm(closest_point - mis_pos_prev)-np.linalg.norm(mis_vec_prev2cur))<0 and dist_to_target<1, dist_to_target, closest_point

###########################################################################################################################################
###########################################################################################################################################
###########################################################################################################################################
###########################################################################################################################################
if __name__ == '__main__':                                     
    missile_pos = np.array([0, 0, 30])
    missile_vel = np.array([0, 0, 15])
    missile_accel_enu = np.array([0, 0, -9.8])*1              # vertical acceleration in ENU frame    
    # HE = np.deg2rad(-20)
    # target_pos = np.array([5, 5, 0])
    # target_vel = np.array([1, 0, 0])
    # target_accl = 0                                       # target acceleration in target frame
    # missile_pos = np.array([30, 0, 0])
    # missile_vel = np.array([0, 0, -1])

    dt = 0.01
    N = 5
    # guidance = Guidance(N, GUIDANCE_TYPE.PURE_PURSUIT)
    guidance = Guidance(N, GUIDANCE_TYPE.PURE_PN)
    # guidance = Guidance(N, GUIDANCE_TYPE.TRUE_PN)
    # guidance = Guidance(N, GUIDANCE_TYPE.ZEM)
    
    nsteps = 1000
    missile_pos_arr = np.zeros((nsteps, 3))
    missile_vel_arr = np.zeros((nsteps, 3))
    accel_cmd_arr = np.zeros((nsteps, 3))
    target_pos_arr = np.zeros((nsteps, 3))
    target_vel_arr = np.zeros((nsteps, 3))
    dist_to_target = np.zeros(nsteps)
    time_arr = np.zeros(nsteps)
    accel_cmd = np.array([0, 0, 0])
    curtime = 0
    
    def targetMovement(t):
        return lissajous_func(t, A=5, B=5, C=4, a=2/2, b=3/2, c=2/2, alt=-1, w = 2 * pi / 10)
       
    for i in range(nsteps):       
        
        (tar_pos, tar_vel, target_accl, tar_3dot, tar_4dot), attitude = targetMovement(curtime)
        time_arr[i]=curtime
        missile_pos_arr[i] = missile_pos
        missile_vel_arr[i] = missile_vel
        accel_cmd_arr[i] = accel_cmd
        target_pos_arr[i] = tar_pos
        target_vel_arr[i] = tar_vel
        dist_to_target[i] = np.linalg.norm(tar_pos - missile_pos)

        if i == 0:
            f0_prev = None
        else:
            f0_prev = np.concatenate((missile_pos_arr[i-1], missile_vel_arr[i-1]))
        
        if i==25:
            pass
        
        f0 = np.concatenate((missile_pos, missile_vel))        
        def integrantFunc(t, y, dt, yprev):
            return calculateTimeDerivative(t=t, y=y, dt=dt, yprev=yprev, tar_pos=tar_pos, tar_vel=tar_vel, tar_accel=target_accl, mis_accel_enu=missile_accel_enu, calcGuidance=True)
        tim, y, dy,_ = advanceRK4(func = integrantFunc, tspan = [curtime, curtime+dt], f0 = f0, dt_prev=dt, f0_prev=f0_prev, numOfSteps = 1)
        
        curtime = tim[-1]
        missile_pos = y[0:3]
        missile_vel = y[3:6]
        event, dist, closest_point = eventFunc(curtime, y, dt, f0_prev, tar_pos)
        
        if  event:        
            event, dist, closest_point = eventFunc(curtime, y, dt, f0_prev, tar_pos)
            break
        
    if event:
        i += 1
        missile_pos_arr[i] = closest_point
        missile_vel_arr[i] = missile_vel
        accel_cmd_arr[i] = accel_cmd
        target_pos_arr[i] = tar_pos
        target_vel_arr[i] = tar_vel
        dist_to_target[i] = dist
 
    # remove from arrays unused elements
    i+=1
    missile_pos_arr = missile_pos_arr[:i]
    missile_vel_arr = missile_vel_arr[:i]
    accel_cmd_arr = accel_cmd_arr[:i]
    target_pos_arr = target_pos_arr[:i]
    target_vel_arr = target_vel_arr[:i]
    dist_to_target = dist_to_target[:i] 
    
    time_arr = time_arr[:i]
    print ("miss distance to target", np.min(np.abs(dist_to_target))) 

    # plot 3D trajectory of missile and target
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    # missile initial position
    ax.plot(missile_pos_arr[0,0], missile_pos_arr[0,1], missile_pos_arr[0,2], 'xr', label='missile initial position')
    ax.plot(missile_pos_arr[:,0], missile_pos_arr[:,1], missile_pos_arr[:,2], label='missile')
    ax.plot(target_pos_arr[0,0], target_pos_arr[0,1], target_pos_arr[0,2], 'or', label='target initial position')
    ax.plot(target_pos_arr[:,0], target_pos_arr[:,1], target_pos_arr[:,2], label='target')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    
    # On second figure plot XY plane trajectory of missile and target
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(missile_pos_arr[:,0], missile_pos_arr[:,1], label='missile')
    ax.plot(target_pos_arr[:,0], target_pos_arr[:,1], label='target')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.legend()
    ax.grid()
    ax.axis('equal')
    
    # On third figure plot time history of missile velocity
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(time_arr[:-1], np.linalg.norm(missile_vel_arr[:-1,:], axis=1), 'x-', label='missile velocity')
    ax.set_xlabel('time')
    ax.set_ylabel('velocity')
    ax.legend()
    ax.grid()
    
    plt.show()
    
    
    pass

    


       