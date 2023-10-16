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

x = np.arange(-15,15,0.1)
y = CM_a_func(x,CM_0,CM_alpha)
fig = plt.figure()
ax1 = fig.add_subplot(111)

ax1.scatter(aero_table.alpha, aero_table.CM, s=10, c='b', marker="s", label='first')
ax1.scatter(x,y, s=10, c='r', marker="o", label='second')

# plt.show()
import env 
import vehicle 

def delta_func(alpha):
    #alpha needs to be in Degrees
    return (-(CM_0 + (CM_alpha * alpha))/CM_delta)

def CL_func(alpha):
    delta = delta_func(alpha)
    return CL_0 + (CL_alpha * alpha) + (CL_delta * delta)


def CD_func(CL):
    return CD_0 + CD_k*(CL**2)

def alpha_calc_func(alpha, velocity,flight_path_angle):

    alpha = alpha[0] # gets rid of depreciatcion of numpy
    alpha_deg = alpha *180/math.pi
    CL = CL_func(alpha_deg)
    CD = CD_func(CL)
    S = vehicle.Sref
    rho = env.air_density

    L = 0.5 * rho * (velocity**2) * S * CL
    D = 0.5 * rho * (velocity**2) * S * CD

    return (-L * math.cos(alpha) - D * math.sin(alpha) + (vehicle.acMass * env.gravity) * math.cos(alpha + flight_path_angle))

velocity = 100
flight_path_angle = 0.05 #rad
# this trails all a ton of values into alpha_calc_func until it returns zero
alpha = optimize.root(alpha_calc_func,x0 = 0.0164, args=(velocity,flight_path_angle))
print(alpha.x[0])
print(alpha_calc_func([0.0],100,0.05))
#print(alpha_calc_func(0.0164,velocity,flight_path_angle))
    
