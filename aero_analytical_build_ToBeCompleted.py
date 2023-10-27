# importing modules
import numpy as np
import math
from scipy import optimize,integrate

import matplotlib.pyplot as plt
import matplotlib.cm as cm

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

        self.theta = self.alpha + self.flight_path_angle
        self.u_b = velocity*math.cos(self.alpha)
        self.w_b = velocity*math.sin(self.alpha)  

    def alpha_calc_func(self,alpha):
        alpha = alpha[0] # gets rid of depreciatcion of numpy

        self.alpha_deg = alpha *180/math.pi
        self.delta_deg = -(CM_0 + (CM_alpha * self.alpha_deg))/CM_delta
        self.delta = self.delta_deg * math.pi/180

        self.CL = CL_0 + (CL_alpha * self.alpha_deg) + (CL_delta * self.delta_deg)
        self.CD = CD_0 + CD_k*(self.CL**2)
        

        self.L = 0.5 * self.rho * (self.Velocity**2) * self.S * self.CL
        self.D = 0.5 * self.rho * (self.Velocity**2) * self.S * self.CD

        return (-self.L * math.cos(alpha) - self.D * math.sin(alpha) + self.W * math.cos(alpha + self.flight_path_angle))

sim = Trim_Simulation(100,0)
print(f"Alpha: {sim.alpha}")
print(f"Delta: {sim.delta}")
print(f"Thrust: {sim.T}")
print(f"Theta: {sim.theta}")
print(f"u_b: {sim.u_b}")
print(f"w_b: {sim.w_b}")
print((sim.L*math.sin(sim.alpha) - sim.D*math.cos(sim.alpha) - sim.W*math.sin(sim.theta) + sim.T))

class Simulation ():
    pass

T = sim.T
PercentageChangeElevator = 10 #percentage
TimeChangeElevator = 2000 #seconds

PercentageChangeThrust = 0 #percentage
TimeChangeThrust = 20 #seconds

def model(t,y):
    if t < 10 or t> 10 + TimeChangeElevator:
        delta = sim.delta
    else:
        delta = sim.delta * (1+ PercentageChangeElevator/100)

    if t < 10 or t> 10 + TimeChangeElevator:
        T = sim.T
    else:
        T = sim.T * (1+ PercentageChangeThrust/100)

    u_b, w_b, theta,q, x_e,z_e = y


    alpha = math.atan(w_b/u_b) 
    V = math.sqrt(u_b**2 + w_b**2)

    W = (vehicle.acMass * env.gravity)
    S = vehicle.Sref
    rho = env.air_density
    cbar = vehicle.cbar

    alpha_deg = alpha *180/math.pi
    delta_deg = delta * 180/math.pi

    CL = CL_0 + (CL_alpha * alpha_deg) + (CL_delta * delta_deg)
    CD = CD_0 + CD_k*(CL**2)
    CM = CM_0 + (CM_alpha * alpha_deg) + (CM_delta * delta_deg)

    L = 0.5 * rho * (V**2) * S * CL
    D = 0.5 * rho * (V**2) * S * CD
    M = 0.5 * rho * (V**2) * S * CM * cbar

    dq_dt = (M/vehicle.inertia_yy)
    dtheta_dt = q

    du_dt = (L*math.sin(alpha) - D*math.cos(alpha) - vehicle.acMass*q*w_b - W*math.sin(theta) + T)/vehicle.acMass
    dw_dt = (-L*math.cos(alpha)-D*math.sin(alpha)+vehicle.acMass*q*u_b + W*math.cos(theta))/vehicle.acMass

    dx_dt = u_b*math.cos(theta) + w_b*math.sin(theta)
    dz_dt = - u_b*math.sin(theta) + w_b*math.cos(theta)

    return du_dt,dw_dt,dtheta_dt,dq_dt,dx_dt,dz_dt

x_e0 = 0
z_e0 = 0
y = integrate.solve_ivp(model,[0,300],[sim.u_b,sim.w_b,sim.theta,0,x_e0,z_e0],t_eval=np.linspace(0,300,3000))

t = y.t

u_b = y.y[0]
w_b = y.y[1]
theta = y.y[2] 
print(theta)
theta = theta * 180/math.pi
print(theta)
q = y.y[3]
x_e = y.y[4]
z_e = y.y[5]
z_e += 2000


fig,ax = plt.subplots(3,2)

ax[0,0].set_ylabel("$u_{B}$",rotation='horizontal')
ax[0,0].set_xlabel("t")
ax[0,1].set_ylabel("$w_{B}$",rotation='horizontal')
ax[0,1].set_xlabel("t")

ax[1,0].set_ylabel("${\Theta}$",rotation='horizontal')
ax[1,0].set_xlabel("t")
ax[1,1].set_ylabel("q",rotation='horizontal')
ax[1,1].set_xlabel("t")

ax[2,0].set_ylabel("$x_{e}$",rotation='horizontal')
ax[2,0].set_xlabel("t")
ax[2,1].set_ylabel("$z_{e}$",rotation='horizontal')
ax[2,1].set_xlabel("t")


ax[0,0].plot(t,u_b)
ax[0,1].plot(t,w_b)
ax[1,0].plot(t,theta)
ax[1,1].plot(t,q)
ax[2,0].plot(t,x_e)
ax[2,1].plot(t,z_e)

# plt.plot(y.t,y.y[0,:] , 'b.-',y.t,y.y[1,:] , 'r-')
# plt.xlabel('x')
# plt.ylabel('y_1(x), y_2(x)')
plt.show()


#--------------------------------------
# B1

fig, (ax1, ax2) = plt.subplots(1, 2)

ax1.set_ylabel("Trust(T)")
ax1.set_xlabel("Velocity(v)")
ax2.set_ylabel("Elevator Angle(δE)")
ax2.set_xlabel("Flight Path Angle(γ)")

for j in np.linspace(-45,45):
    y = []
    x = []
    for i in np.linspace(0,100):
        sim = Trim_Simulation(i,j/100)
        if sim.T>0 and (sim.delta_deg>-20 and sim.delta_deg<20) and (sim.alpha_deg>-16 and sim.alpha_deg<12):
            y.append(sim.T)
            x.append(i)
        

    ax1.plot(x,y,label = f"Gamma: {j}",c=cm.hot(j/50))

for j in np.linspace(0,1000):
    
    y = []
    x = []
    for i in np.linspace(-45,45):
        sim = Trim_Simulation(j,i/100)
        if sim.T>0 and (sim.delta_deg>=-20 and sim.delta_deg<=20) and (sim.alpha_deg>=-16 and sim.alpha_deg<=12):
            x.append(i/100)
            y.append(sim.delta*180/math.pi)

    ax2.plot(x,y,label = f"Velcities: {j}")


# plt.show()

#--------------------------------------

##B2
TrimCondition1 = Trim_Simulation(119,0.0)
TrimCondition2 = Trim_Simulation(119,2*math.pi/180)
TrimCondition3 = Trim_Simulation(119,0.0)

