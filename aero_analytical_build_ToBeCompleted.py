# importing modules
import numpy as np
import math
from scipy import optimize
import matplotlib.pyplot as plt

#--------------------------------------
# importing module with aerodynamics model 
# as tables of CL CM vs alpha
#    tables of CD vs CL
# and tables of CL_el and CM_el vs delta_el 
import aero_table
#--------------------------------------

#--------------------------------------
# Lift vs alpha

# Initial guesses for the fitting
# i.e., initial values of a and b
CL_0 = 0.0410
CL_alpha = 0.1

# Functional form to fit to data
def CL_a_func(x, a, b):
    return a + b * x

# Fitting (find a and b of the above function)  
# using the python module optimize from scipy.
# params contains the fitted values of a and b
# params_covariance contains a measure of the achieved 
# accuracy 
params, params_covariance = optimize.curve_fit(CL_a_func, aero_table.alpha, aero_table.CL, p0=[CL_0, CL_alpha])

CL_0 = params[0]
CL_alpha = params[1]
#--------------------------------------

#--------------------------------------
#Lift vs delta_elevator
CL_delta = 0.003

def CL_d_func(x, a):
    return a * x

params, params_covariance = optimize.curve_fit(CL_d_func, aero_table.delta_el, aero_table.CL_el, p0=[CL_delta])

CL_delta = params[0]
#--------------------------------------
#--------------------------------------
# CD vs CL
CD_0 = 0.026
CD_k = 0.045

def CD_CL_func(x, a, b):
    return a + b * x**2.0

params, params_covariance = optimize.curve_fit(CD_CL_func, aero_table.CL, aero_table.CD, p0=[CD_0, CD_k])

CD_0 = params[0]
CD_k = params[1]
#--------------------------------------

#--------------------------------------
# Moment vs alpha
CM_0 = 0.
CM_alpha = -0.01

# TO BE COMPLETED HERE
#--------------------------------------

# Functional form to fit to data
def CM_a_func(x, a, b):
    return a + b * x

params, params_covariance = optimize.curve_fit(CM_a_func, aero_table.alpha, aero_table.CM, p0=[CM_0, CM_alpha])

CM_0 = params[0]
CM_alpha = params[1]

#--------------------------------------
#Moment vs delta_elevator
CM_delta = -0.004

# TO BE COMPLETED HERE
#--------------------------------------
def CM_d_func(x, a):
    return a * x


params, params_covariance = optimize.curve_fit(CM_d_func, aero_table.delta_el, aero_table.CM_el, p0=[CM_delta])

CM_delta = params[0]

#--------------------------------------
# Write results on screen (check)
print(CL_0, CL_alpha, CL_delta)
print(CD_0,CD_k)
print(CM_0, CM_alpha, CM_delta)
#--------------------------------------


# plt.show()
import env 
import vehicle 


class Trim_Simulation:
    def __init__(self,velocity,flight_path_angle):
        self.Velocity = velocity
        self.flight_path_angle = flight_path_angle
        self.W = (vehicle.acMass * env.gravity)
        self.S = vehicle.Sref
        self.rho = env.air_density

        alpha = optimize.root(self.alpha_calc_func,x0 = 0, args=())
        self.alpha = alpha.x[0]

        self.T = self.W * math.sin(self.alpha + flight_path_angle) + self.D * math.cos(self.alpha) - self.L * math.sin(self.alpha)

        self.theta = self.alpha + self.delta
        self.u_b = velocity*math.cos(self.alpha)
        self.w_b = velocity*math.sin(self.alpha)  

    def alpha_calc_func(self,alpha):
        alpha = alpha[0] # gets rid of depreciatcion of numpy

        alpha_deg = alpha *180/math.pi
        delta_deg = -(CM_0 + (CM_alpha * alpha_deg))/CM_delta
        self.delta = delta_deg * math.pi/180

        self.CL = CL_0 + (CL_alpha * alpha_deg) + (CL_delta * delta_deg)
        self.CD = CD_0 + CD_k*(self.CL**2)
        

        self.L = 0.5 * self.rho * (self.Velocity**2) * self.S * self.CL
        self.D = 0.5 * self.rho * (self.Velocity**2) * self.S * self.CD

        return (-self.L * math.cos(alpha) - self.D * math.sin(alpha) + self.W * math.cos(alpha + self.flight_path_angle))

sim = Trim_Simulation(100,0.05)
print(f"Alpha: {sim.alpha}")
print(f"Delta: {sim.delta}")
print(f"Thrust: {sim.T}")
print(f"Theta: {sim.theta}")
print(f"u_b: {sim.u_b}")
print(f"w_b: {sim.w_b}")
