import time
import numpy as np
import torch
from rl_policy import RLPolicy

from math import sin, cos, pi, sqrt, atan2

        
###############################################################################################################
class VelocityRLController:

    def __init__(self, maximalVelocity=3.0):

        self.controllerType = "VelocityRL"
        self.dt = 0.01

        # state space parameters
        self.pos_self = [0.0,0.0,0.0] # NED
        self.yaw = 0.0                # relative to North
        self.pos_target = [0.0,0.0,0.0]
        self.target_distance = 0.0    # relative to self
        self.target_distance_dot = 0.0
        self.last_distance = 0.0
        self.target_angle = 0.0       # relative to Forward
        self.max_vel = maximalVelocity

        # policy network setup
        self.policy = RLPolicy.load_from_checkpoint(
            "best_step_res.pth",  ## CHANGE TO RELEVANT MODEL NAME
            device="cpu"
        ).eval()
        # initial GRU hidden state
        self.hxs = torch.zeros(1, 1, 512)
        
###############################################################################################################
    def getCommand(self, currentBodyState, desiredBodyState, controlType=None, currentData=None):
               
        ## get raw position data
        self.pos_self, _, _, _, _ = currentBodyState
        self.pos_target, _, _, _, _ = desiredBodyState[0]
        if currentData is not None:
            self.yaw=currentData.rpy[2]
        
        ## transform into observation space data
        delta = self.pos_target - self.pos_self
        self.target_distance = np.linalg.norm(delta)
        if self.target_distance > 20.0: self.target_distance = 20.0
        self.target_angle = atan2(delta[1],delta[0])
        self.target_angle = (self.target_angle - self.yaw + pi) % (2*pi) - pi
        self.target_distance_dot = (self.last_distance - self.target_distance)/self.dt
        self.last_distance = self.target_distance
        obs = np.array([self.target_distance, self.target_angle, self.target_distance_dot],dtype=np.float32)
        obs_tensor = torch.from_numpy(obs).unsqueeze(0)
        
        ## execute policy inference
        with torch.no_grad():
            mean_action, self.hxs = self.policy(obs_tensor, self.hxs)
            vf,vr,w = mean_action.squeeze(0).numpy()  # [v_r, v_θ, w_yaw]
            vf = np.clip(vf,-self.max_vel,self.max_vel)
            vr = np.clip(vr,-self.max_vel,self.max_vel)
        
        ## vectorize and send outputs
        vel_vector = [vf,vr,0] # in FRD
        omega_vector = [0,0,w]
        # print("velocity  "+str(vf)+" "+str(vr)+"  omega "+str(w))
        return vel_vector, np.eye(3), omega_vector

    



