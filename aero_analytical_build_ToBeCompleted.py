import numpy as np
import math
from scipy import optimize
import aero_table
import matplotlib.pyplot as plt
from scipy.optimize import newton
from scipy.integrate import solve_ivp

# Define initial parameters
CL_0 = 0.0410
CL_alpha = 0.1
CL_delta = 0.003
CD_0 = 0.026
CD_k = 0.045
CM_0 = 0.0
CM_alpha = -0.01
CM_delta = -0.004

class Vehicle:
    Sref = 20

class Environment:
    air_density = 1.0065
    gravity = 9.81

vehicle = Vehicle()
env = Environment()

# Define functions for curve fitting
def CL_a_func(x, a, b):
    return a + b * x

def CL_d_func(x, a):
    return a * x

def CD_CL_func(x, a, b):
    return a + b * x**2.0

def CM_a_func(x, a, b):
    return a + b * x

def CM_d_func(x, a):
    return a * x

# Curve fitting
params, params_covariance = optimize.curve_fit(CL_a_func, aero_table.alpha, aero_table.CL, p0=[CL_0, CL_alpha])
CL_0 = params[0]
CL_alpha = params[1]

params, params_covariance = optimize.curve_fit(CL_d_func, aero_table.delta_el, aero_table.CL_el, p0=[CL_delta])
CL_delta = params[0]

params, params_covariance = optimize.curve_fit(CD_CL_func, aero_table.CL, aero_table.CD, p0=[CD_0, CD_k])
CD_0 = params[0]
CD_k = params[1]

params, params_covariance = optimize.curve_fit(CM_a_func, aero_table.alpha, aero_table.CM, p0=[CM_0, CM_alpha])
CM_0 = params[0]
CM_alpha = params[1]

params, params_covariance = optimize.curve_fit(CM_d_func, aero_table.delta_el, aero_table.CM_el, p0=[CM_delta])
CM_delta = params[0]

# Write results on screen (check)
print(CL_0, CL_alpha, CL_delta)
print(CD_0,CD_k)
print(CM_0, CM_alpha, CM_delta)

# Define functions for calculations
def delta_func(alpha):
    alpha_deg = alpha * 180/math.pi
    return (-(CM_0 + (CM_alpha * alpha_deg)) / CM_delta)

def CL_func(alpha):
    alpha_deg = alpha * 180/math.pi
    delta_local = delta_func(alpha)  # Use a different variable name
    return CL_0 + CL_alpha * alpha_deg + CL_delta * delta_local

def CD_func(CL):
    return CD_0 + CD_k * (CL ** 2)

def CM_func(alpha):
    alpha_deg = alpha * 180/math.pi
    delta_local = delta_func(alpha)  # Use a different variable name
    return CM_0 + CM_alpha * alpha_deg + CM_delta * delta_local

def alpha_calc_func(alpha, velocity, flight_path_angle):
    CL = CL_func(alpha)
    CD = CD_func(CL)
    S = vehicle.Sref
    rho = env.air_density
    L = 0.5 * rho * (velocity ** 2) * S * CL
    D = 0.5 * rho * (velocity ** 2) * S * CD

    return -L * math.cos(alpha) - D * math.sin(alpha) + (1300 * env.gravity) * math.cos(alpha + flight_path_angle)

velocity = 100
flight_path_angle = 0.0
target_alpha = 0.0164

def alpha_Calc(alpha):
    return alpha_calc_func(alpha, velocity, flight_path_angle) - target_alpha

def thrust_calc_func(alpha, flight_path_angle):
    CL = CL_func(alpha)
    CD = CD_func(CL)
    S = vehicle.Sref
    rho = env.air_density
    alpha_deg = alpha * 180/math.pi
    L = 0.5 * rho * (velocity ** 2) * S * CL
    D = 0.5 * rho * (velocity ** 2) * S * CD

    return (1300 * env.gravity) * math.sin(alpha + flight_path_angle) + D * math.cos(alpha) - L * math.sin(alpha)

def elev_calc(alpha):
    CM = CM_func(alpha)
    S = vehicle.Sref
    rho = env.air_density
    alpha_deg = alpha * 180/math.pi

    return -(CM_0 + CM_alpha * alpha)/CM_delta

alpha = newton(alpha_Calc, 0.0164, tol=0.001, maxiter=100)
print('alpha =', alpha, 'rad')

thrust = thrust_calc_func(alpha, flight_path_angle)
print('Thrust =', thrust, 'N')

delta_local = delta_func(alpha) * math.pi / 180  # Use a different variable name
print('Delta =', delta_local, 'rad')

pitch_angle = flight_path_angle + alpha
print('Pitch angle =', pitch_angle, 'rad')

u_b = velocity * math.cos(alpha)
w_b = velocity * math.sin(alpha)
print('u_b =', u_b, 'm/s')
print('w_b =', w_b, 'm/s')


# importing modules
import numpy as np
import math

# Airplane Characteristics

# Wing surface
Sref = 20.0  

# airfoil chord
cbar = 1.75 

# Mass of the airplane
acMass = 1300.0 

# Moment of inertia
inertia_yy = 7000



# Commands before trim (only a definitionion, the numbers do not matter)
# After trim, these variable store the trim value
Thrust = 1.0
delta_el = 0.0


# Dummy definition of coeff
CL = 0.0
CD = 0.0
CM = 0.0

L = 0.0
D = 0.0
M = 0.0

#Trimmed ALpha
from scipy import optimize
from scipy.optimize import newton
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# Import the aerodynamics data
import aero_table

# Fitting function for CL vs alpha
CL_0 = 0.0410
CL_alpha = 0.1

def CL_a_func(x, a, b):
    return a + b * x

params, params_covariance = optimize.curve_fit(CL_a_func, aero_table.alpha, aero_table.CL, p0=[CL_0, CL_alpha])

CL_0 = params[0]
CL_alpha = params[1]

# Fitting function for CL vs delta_elevator
CL_delta = 0.003

def CL_d_func(x, a):
    return a * x

params, params_covariance = optimize.curve_fit(CL_d_func, aero_table.delta_el, aero_table.CL_el, p0=[CL_delta])

CL_delta = params[0]

# Fitting function for CD vs CL
CD_0 = 0.026
CD_k = 0.045

def CD_CL_func(x, a, b):
    return a + b * x**2.0

params, params_covariance = optimize.curve_fit(CD_CL_func, aero_table.CL, aero_table.CD, p0=[CD_0, CD_k])

CD_0 = params[0]
CD_k = params[1]

# Fitting function for CM vs alpha
CM_0 = 0.0
CM_alpha = -0.01

def CM_a_func(x, a, b):
    return a + b * x

params, params_covariance = optimize.curve_fit(CM_a_func, aero_table.alpha, aero_table.CM, p0=[CM_0, CM_alpha])

CM_0 = params[0]
CM_alpha = params[1]

# Fitting function for CM vs delta_elevator
CM_delta = -0.004

def CM_d_func(x, a):
    return a * x

params, params_covariance = optimize.curve_fit(CM_d_func, aero_table.delta_el, aero_table.CM_el, p0=[CM_delta])

CM_delta = params[0]

# Define the vehicle and environment
class Vehicle:
    Sref = 20  # Sample reference area

class Environment:
    air_density = 1.0065  # Sample air density
    gravity = 9.81

vehicle = Vehicle()
env = Environment()

# Function to calculate elevator deflection based on alpha
def delta_func(alpha):
    alpha_deg = alpha * 180 / math.pi
    return (-(CM_0 + (CM_alpha * alpha_deg)) / CM_delta)

# Function to calculate CL based on alpha and elevator deflection
def CL_func(alpha, delta):
    alpha_deg = alpha * 180 / math.pi
    delta_deg = delta * 180 / math.pi
    return CL_0 + CL_alpha * alpha_deg + CL_delta * delta_deg

# Function to calculate CD based on CL
def CD_func(CL):
    return CD_0 + CD_k * (CL ** 2)

# Function to calculate CM based on alpha and elevator deflection
def CM_func(alpha, delta):
    alpha_deg = alpha * 180 / math.pi
    delta_deg = delta * 180 / math.pi
    return CM_0 + CM_alpha * alpha_deg + CM_delta * delta_deg

# Function to calculate alpha based on aircraft state
def alpha_calc_func(alpha, velocity, flight_path_angle, delta):
    CL = CL_func(alpha, delta)
    CD = CD_func(CL)
    S = vehicle.Sref
    rho = env.air_density
    L = 0.5 * rho * (velocity ** 2) * S * CL
    D = 0.5 * rho * (velocity ** 2) * S * CD

    return -L * math.cos(alpha) - D * math.sin(alpha) + (1300 * env.gravity) * math.cos(alpha + flight_path_angle)

velocity = 100
flight_path_angle = 0.0  # rad
target_alpha = 0.0164  # The desired value

# Define a custom function for root-finding
def alpha_Calc(alpha):
    return alpha_calc_func(alpha, velocity, flight_path_angle, 0.0) - target_alpha

# Calculate the trim alpha value
alpha = newton(alpha_Calc, 0.0164, tol=0.001, maxiter=100)
print('Trimmed alpha =', alpha, 'rad')

# Calculate and print the thrust
def thrust_calc_func(alpha, flight_path_angle, delta):
    CL = CL_func(alpha, delta)
    CD = CD_func(CL)
    S = vehicle.Sref
    rho = env.air_density
    alpha_deg = alpha * 180 / math.pi
    L = 0.5 * rho * (velocity ** 2) * S * CL
    D = 0.5 * rho * (velocity ** 2) * S * CD
    return (1300 * env.gravity) * math.sin(alpha + flight_path_angle) + D * math.cos(alpha) - L * math.sin(alpha)

thrust = thrust_calc_func(alpha, flight_path_angle, 0.0)
print('Thrust =', thrust, 'N')

# Calculate and print delta in radians
delta = delta_func(alpha)
print('Elevator Deflection (Delta) =', delta, 'rad')

# Calculate and print pitch angle
pitch_angle = flight_path_angle + alpha
print('Pitch Angle =', pitch_angle, 'rad')

# Calculate and print u_b and w_b
u_b = velocity * math.cos(alpha)
w_b = velocity * math.sin(alpha)
print('u_b =', u_b, 'm/s')
print('w_b =', w_b, 'm/s')

# Data for plotting
alpha_values = np.linspace(-0.2, 0.2, 100)  # Range of alpha values for plotting
CL_values = [CL_func(alpha, delta) for alpha in alpha_values]
CD_values = [CD_func(CL) for CL in CL_values]
CM_values = [CM_func(alpha, delta) for alpha in alpha_values]

plt.figure(figsize=(12, 4))

plt.subplot(131)
plt.plot(alpha_values, CL_values)
plt.xlabel('Alpha (radians)')
plt.ylabel('CL')
plt.title('CL vs. Alpha')

plt.subplot(132)
plt.plot(CL_values, CD_values)
plt.xlabel('CL')
plt.ylabel('CD')
plt.title('CD vs. CL')

plt.subplot(133)
plt.plot(alpha_values, CM_values)
plt.xlabel('Alpha (radians)')
plt.ylabel('CM')
plt.title('CM vs. Alpha')

plt.tight_layout()
plt.show()

# A3 Differential Equations
import math
import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt


delta = -0.0520  # Initial elevator angle
T = 2755.15413277993
TimeChangeElevator = 2000  # Time when elevator angle changes
PercentageChangeElevator = 20  # Percentage change in elevator angle
PercentageChangeThrust = 0  # Percentage change in thrust (set to 0 for no change)


def model(t, y, delta, T):
    if t < 10 or t> 10 + TimeChangeElevator:
        delta = -0.0624
    else:
        delta = delta * (1+ PercentageChangeElevator/100)
    if t < 10 or t > 10 + TimeChangeElevator:
        T = thrust
    else:
        T = thrust * (1 + PercentageChangeThrust / 100)
    
    u_b, w_b, theta, q, x_e, z_e = y

    alpha = math.atan(w_b / u_b)
    V = math.sqrt(u_b ** 2 + w_b ** 2)

    W = (1300 * env.gravity)
    S = vehicle.Sref
    rho = env.air_density
    alpha_deg = alpha * 180 / math.pi
    delta_deg = delta * 180 / math.pi

    CL = CL_0 + CL_alpha * alpha_deg + CL_delta * delta_deg
    CD = CD_0 + CD_k * (CL ** 2)
    CM = CM_0 + CM_alpha * alpha_deg + CM_delta * delta_deg

    L = 0.5 * rho * (V ** 2) * S * CL
    D = 0.5 * rho * (V ** 2) * S * CD
    M = 0.5 * rho * (V ** 2) * S * CM * cbar

    dq_dt = M / inertia_yy
    dtheta_dt = q

    du_dt = (L * math.sin(alpha) - D * math.cos(alpha) - acMass * q * w_b - W * math.sin(theta) + thrust) / acMass
    dw_dt = (-L * math.cos(alpha) - D * math.sin(alpha) + acMass * q * u_b + W * math.cos(theta)) / acMass

    dx_dt = u_b * math.cos(theta) + w_b * math.sin(theta)
    dz_dt = -u_b * math.sin(theta) + w_b * math.cos(theta)

    return du_dt, dw_dt, dtheta_dt, dq_dt, dx_dt, dz_dt

# Initial conditions
u_b0 = velocity * math.cos(alpha)
w_b0 = velocity * math.sin(alpha)
theta0 = 0.0
q0 = 0.0
x_e0 = 0.0
z_e0 = 0.0

# Solve the system of differential equations
y = solve_ivp(model, [0, 300], [u_b0, w_b0, theta0, q0, x_e0, z_e0], t_eval=np.linspace(0, 300, 3000), args=(delta, T))

t = y.t
u_b = y.y[0]
w_b = y.y[1]
theta = y.y[2]
q = y.y[3]
x_e = y.y[4]
z_e = y.y[5]
z_e += 2000


plt.figure(figsize=(12, 8))

plt.subplot(2, 3, 1)
plt.plot(t, u_b)
plt.xlabel('Time')
plt.ylabel('u_b')
plt.grid(True)

plt.subplot(2, 3, 2)
plt.plot(t, w_b)
plt.xlabel('Time')
plt.ylabel('w_b')
plt.grid(True)

plt.subplot(2, 3, 3)

plt.plot(t, [math.degrees(angle) for angle in theta])
plt.xlabel('Time')
plt.ylabel('theta (degrees)')
plt.grid(True)

plt.subplot(2, 3, 4)
plt.plot(t, q)
plt.xlabel('Time')
plt.ylabel('q')
plt.grid(True)

plt.subplot(2, 3, 5)
plt.plot(t, x_e)
plt.xlabel('Time')
plt.ylabel('x_e')
plt.grid(True)

plt.subplot(2, 3, 6)
plt.plot(t, z_e)
plt.xlabel('Time')
plt.ylabel('z_e')
plt.grid(True)

plt.tight_layout()
plt.show()
