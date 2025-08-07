from matrix_utils import hat, vee, deriv_unit_vector, saturate
from integral_utils import IntegralError, IntegralErrorVec3

import time
import numpy as np

from math import sin, cos, pi, sqrt

###############################################################################################################
class VelocityPIDControllerParameters:
    def __init__(self, mass=0.5):
        self.mass = mass  # Mass of the rover (kg)

            # controller
        # Position gains
        commonFactor = 1
        self.kX = np.diag([1.0, 1.0, 1.0])*commonFactor  # Position gains
        self.kV = np.diag([3.0, 3.0, 1.0])*commonFactor # Velocity gains
        # Integral gains
        self.kIX = np.diag([0, 0, 0])*commonFactor # Position integral gains
        self.kIV = np.diag([1, 1, 1])*0 # Velocity integral gains

        self.c1 = 1.0  # Parameters for Position decoupled-yaw integral control
        
        self.sat_sigmaX = 3
        self.sat_sigmaV = 3
        
###############################################################################################################
class VelocityPIDController:

    def __init__(self, mass):

        self.controllerType = "VelocityPID"
        self.t0 = time.time()
        self.t = 0.0
        self.t_pre = 0.0
        self.dt = 1e-9

        # Flag to enable/disable integral control 
        self.use_integralTerm = True

        self.A = np.zeros((3, 3))

        self.param = VelocityPIDControllerParameters(mass=mass)

            # controller
        # Control errors
        self.ex = np.zeros(3)
        self.ev = np.zeros(3)
        
        # Integral errors
        self.eIX = IntegralErrorVec3()  # Position integral error
        self.eIV = IntegralErrorVec3()  # Velocity integral error

        # derivative errors
        self.eDV = np.zeros(3)      # Derivative of velocity error

###############################################################################################################
    def resetIntegralErrorTerms(self):
        self.eIX.set_zero()
        self.eIV.set_zero()
        
###############################################################################################################
    def getCommand(self, currentBodyState, desiredBodyState, controlType=None, currentData=None):
        if controlType is None:
            pos_control=True
            vel_control=True
        else:
            pos_control = controlType[0]
            vel_control = controlType[1]
            
        (pos_ned, vel_ned, accel_ned, omega_ned, quat_ned_bodyfrd) = currentBodyState

        (xd, xd_dot, xd_2dot, xd_3dot, xd_4dot) = desiredBodyState[0]
        (b1d, b1d_dot, b1d_2dot) = desiredBodyState[1] 

        kX = self.param.kX if (pos_control and xd is not None) else np.zeros((3,3))
        kIX = self.param.kIX if (pos_control and xd is not None) else np.zeros((3,3))
        kV = self.param.kV #if (vel_control and xd_dot is not None) else np.zeros((3,3))
        kIV = self.param.kIV if (vel_control and xd_dot is not None) else np.zeros((3,3))

        self.update_current_time()
        self.dt = self.t - self.t_pre

        # Translational error functions
        
        eX = (xd - pos_ned)     if (pos_control and xd is not None)     else np.zeros(3) # position error - eq (11)
        eV = (xd_dot - vel_ned) #if (vel_control and xd_dot is not None) else np.zeros(3) # velocity error - eq (12)

        # Position integral terms
        if self.use_integralTerm:
            self.eIX.integrate(self.param.c1 * eX + eV, self.dt)                 # eq (13)
            self.eIX.error = saturate(self.eIX.error, -self.param.sat_sigmaX, self.param.sat_sigmaX)
            self.eIV.integrate(eV, self.dt)                 # eq (13)
            self.eIV.error = saturate(self.eIV.error, -self.param.sat_sigmaV, self.param.sat_sigmaV)

        A =   kX @ eX \
            + kV @ eV \
            + kIX @ self.eIX.error \
            + kIV @ self.eIV.error \
            + xd_2dot# Velocity Command
            

        self.A = A
        return A, np.eye(3), np.zeros(3)
###############################################################################################################

    def update_current_time(self):
        """Update the current time since epoch."""
        self.t_pre = self.t

        self.t = time.time() - self.t0
    



