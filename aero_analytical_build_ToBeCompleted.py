#--------------------------------------  Importing Modules  ---------------------------------------------------------------------------
import numpy as np
import math
from scipy import optimize,integrate
import pandas as pd 
import matplotlib
matplotlib.use("macosx")
import matplotlib.pyplot as plt 

#--------------------------------------
# importing module with aerodynamics model 
# as tables of CL CM vs alpha
#    tables of CD vs CL
# and tables of CL_el and CM_el vs delta_el 
import aero_table

#Imports necessary variables related to the vehicle and environment
import env 
import vehicle 

#--------------------------------------  Lift vs alpha  -------------------------------------------------------------------------------

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

#--------------------------------------  Lift vs delta_elevator  ----------------------------------------------------------------------

CL_delta = 0.003

def CL_d_func(x, a):
    return a * x

params, params_covariance = optimize.curve_fit(CL_d_func, aero_table.delta_el, aero_table.CL_el, p0=[CL_delta])

CL_delta = params[0]

#--------------------------------------  CD vs CL  ------------------------------------------------------------------------------------

CD_0 = 0.026
CD_k = 0.045

def CD_CL_func(x, a, b):
    return a + b * x**2.0

params, params_covariance = optimize.curve_fit(CD_CL_func, aero_table.CL, aero_table.CD, p0=[CD_0, CD_k])

CD_0 = params[0]
CD_k = params[1]

#--------------------------------------  Moment vs alpha  -----------------------------------------------------------------------------

CM_0 = 0.
CM_alpha = -0.01

def CM_a_func(x, a, b):
    return a + b * x

params, params_covariance = optimize.curve_fit(CM_a_func, aero_table.alpha, aero_table.CM, p0=[CM_0, CM_alpha])

CM_0 = params[0]
CM_alpha = params[1]

#--------------------------------------  Moment vs delta_elevator  --------------------------------------------------------------------

CM_delta = -0.004

def CM_d_func(x, a):
    return a * x

params, params_covariance = optimize.curve_fit(CM_d_func, aero_table.delta_el, aero_table.CM_el, p0=[CM_delta])

CM_delta = params[0]

#-----------------------------------------------    A2    ----------------------------------------------------------------------------

class Trim_Simulation:
    def __init__(self,velocity:float,flight_path_angle:float):
        """
        Calculates all Trim Conditions and stored them
        
        self.alpha      (Radians)
        self.delta      (Radians)
        self.T          (N)
        self.theta      (Radians)
        self.u_b        (Meters per second)
        self.w_b        (Meters per second)
        """
        #Initializes class Variables
        self.Velocity = velocity
        self.flight_path_angle = flight_path_angle
        self.W = (vehicle.acMass * env.gravity)
        self.S = vehicle.Sref
        self.rho = env.air_density

        # Finding Alpha 
        alpha = optimize.root(self.alpha_calc_func,x0 = 0, args=()) # Ittereates through alpha_calc_func until it returns 0
        self.alpha = alpha.x[0]

        #Finding remaining values using alpha
        self.T = self.W * math.sin(self.alpha + flight_path_angle) + self.D * math.cos(self.alpha) - self.L * math.sin(self.alpha)
        
        self.theta = self.alpha + self.flight_path_angle
        self.u_b = velocity*math.cos(self.alpha)
        self.w_b = velocity*math.sin(self.alpha)  

    def alpha_calc_func(self,alpha):
        """
        Uses trim condition equations to find dw/dt for a given alpha
        """
        alpha = alpha[0] # gets rid of depreciatcion of numpy

        self.alpha_deg = alpha *180/math.pi
        self.delta_deg = -(CM_0 + (CM_alpha * self.alpha_deg))/CM_delta
        self.delta = self.delta_deg * math.pi/180

        self.CL = CL_0 + (CL_alpha * self.alpha_deg) + (CL_delta * self.delta_deg)
        self.CD = CD_0 + CD_k*(self.CL**2)
        

        self.L = 0.5 * self.rho * (self.Velocity**2) * self.S * self.CL
        self.D = 0.5 * self.rho * (self.Velocity**2) * self.S * self.CD

        return (-self.L * math.cos(alpha) - self.D * math.sin(alpha) + self.W * math.cos(alpha + self.flight_path_angle))

#-----------------------------------------------    A3    ----------------------------------------------------------------------------

# General Class inherited by anything using the simulation
class Simulation():
    
    def PlotData(self):
        fig, ax = plt.subplots(4, 2, figsize=(16, 16))

        fig.suptitle("Longitudinal Flight Dynamics", fontsize=36)

        fig = self.SetTitles(fig)

        t_label = 'time, t [s]'
        
        y_labels = ["velocity, $u_{B}$ [m$\mathregular{s^{-1}}$]",
                    "velocity, $w_{B}$ [m$\mathregular{s^{-1}}$]",
                    "angular velocity, q [rad$\mathregular{s^{-1}}$]",
                    "pitch angle, ${\Theta}$ [°]",
                    "flight path angle, ${\u03B3}$ [°]",
                    "displacement, $z_{e}$ [m]",
                    "alpha, ${\u03B1}$ [°]",
                    "Altitude, h [m]"]
        
        y_axis = [self.u_b, self.w_b, self.q, self.theta_deg, self.gamma_deg, self.z_e,self.alpha_deg,self.altitude]
        
        for i in range(4):
            for j in range(2):
                ax[i, j].set_ylabel(y_labels[i * 2 + j],)
                ax[i, j].set_xlabel(t_label)
                ax[i, j].plot(self.t, y_axis[i * 2 + j], color='black')
                ax[i, j].grid(linestyle='-', color='black')
    
        plt.show()
    
    def SetTitles(self,fig):
        pass

    def HandleSimulationData(self,Data,initial_h = 2000):
        """
        Takes data from solve_ivp and puts then into variables for plotting or GUI
        """
        self.t = Data.t

        self.u_b = Data.y[0]
        self.w_b = Data.y[1]
        self.theta = Data.y[2] 
        self.theta_deg = self.theta * 180/math.pi
        self.q = Data.y[3]
        self.x_e = Data.y[4]
        self.z_e = Data.y[5] - initial_h

        self.alpha = np.arctan(self.w_b/self.u_b) #Numpy.arctan is needed for nparrays
        self.alpha_deg = self.alpha * 180/math.pi
        self.gamma = self.theta - self.alpha
        self.gamma_deg = self.gamma * 180/math.pi

        self.altitude = self.z_e * -1

    def Calculations(self,t,y,delta,T):
        """
        Paramters
        ----------
        t : Time step (seconds)
        y : array of values for each variable 
            u_b : (m/s)
            w_b : (m/s)
            theta : (Radians)
            q : (Radians per second)
            x_e : (Meters)
            z_e : (Meters)
        delta: Elevator angle (Radians)
        T: Thrust (N)

        Returns
        ----------
        The calculated change in each variable in y over the time period
        """
        u_b, w_b, theta,q, x_e,z_e = y #Unpacks Variables from y

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

## For user Interface 
class CustomSimulation (Simulation):
    def __init__(self,TrimV,TrimGamma,TotalSimulationTime,PercentageChangeElevator = 0,TimeChangeElevator = 0,PercentageChangeThrust = 0,TimeChangeThrust=0):
        """
        Finds the response of the simulation to a change in thrust and elevator angle after a set amount of time
        """
        Trim = Trim_Simulation(TrimV,TrimGamma)
        self.Trim = Trim
        self.PercentageChangeElevator = PercentageChangeElevator # The amount Elevator Angle changes by
        self.TimeChangeElevator = TimeChangeElevator # The time at which Elevator Angle changes
        self.PercentageChangeThrust = PercentageChangeThrust # The amount Thrust changes by
        self.TimeChangeThrust = TimeChangeThrust # The time at which Thrust changes

        y = integrate.solve_ivp(self.Model,[0,TotalSimulationTime],[Trim.u_b,Trim.w_b,Trim.theta,0,0,0],t_eval=np.linspace(0,TotalSimulationTime,TotalSimulationTime*100))
        self.HandleSimulationData(y)
        self.PlotData()
       
    def Model(self,t,y):
        # waits until the designated time change and applies the percentage change for either thrust or Elevator angle or both
        if t > self.TimeChangeElevator:
            delta = self.Trim.delta * (100+ self.PercentageChangeElevator)/100
        else:
            delta = self.Trim.delta
        
        if t > self.TimeChangeThrust:
            Thrust = self.Trim.T * (100 + self.PercentageChangeThrust)/100
        else:
            Thrust = self.Trim.T

        return self.Calculations(t,y,delta,Thrust)

    def SetTitles(self, fig):
        plt.subplots_adjust(top=0.81,bottom=0.075)
        fig.text(s="A3: Trim Conditions", x=0.5, y=0.9, fontsize=20, ha='center', va='center')
        fig.text(s='V = 100 ms-1, γ=0.0°', x=0.5, y=0.87, fontsize=12, ha='center', va='center')
        fig.text(s="At t = 100 s , $δ_{E}$ is increased by 10% from -0.0520° to -0.0572°", x=0.5, y=0.84, fontsize=12, ha='center', va='center')
        return fig

#-----------------------------------------------    B1    ----------------------------------------------------------------------------

def B1():
    fig, ax = plt.subplots(2, 2, figsize=(30,20))
    plt.subplots_adjust(top=0.8,bottom=0.075)
    
    # Getting Data
    for j in range(-2,3):
        Thrust = []
        Delta = []
        Velocity = []
        for i in np.linspace(0,120,5000):
            sim = Trim_Simulation(i,j*math.pi/180)
            if sim.T>0 and (sim.delta_deg>=-20 and sim.delta_deg<=20) and (sim.alpha_deg>=-16 and sim.alpha_deg<=12):
                
                Thrust.append(sim.T)
                Delta.append(sim.delta_deg)
                Velocity.append(i)

        ax[0,0].plot(Velocity,Thrust,label=f"γ = {j}°")
        ax[1,0].plot(Velocity,Delta,label=f"γ = {j}°")
        
    for j in range(-2,3):
        Delta = []
        Thrust = []
        Flight_Path_Angle = []
        for i in np.linspace(-90,10,5000):
            sim = Trim_Simulation(90 + j*10,i*math.pi/180)
            if sim.T>0 and (sim.delta_deg>=-20 and sim.delta_deg<=20) and (sim.alpha_deg>=-16 and sim.alpha_deg<=12):
                Delta.append(sim.delta_deg)
                Thrust.append(sim.T)
                Flight_Path_Angle.append(i)
                
        ax[0,1].plot(Flight_Path_Angle,Thrust,label = f"v = {90 + j*10} m/s")
        ax[1,1].plot(Flight_Path_Angle,Delta,label = f"v = {90 + j*10} m/s")

    ###Setting up graphs
    fig.suptitle("Longitudinal Flight Dynamics", fontsize=36)
    fig.text(s="B1: Engineering Design Simulations", x=0.5, y=0.89, fontsize=20, ha='center', va='center')
    fig.text(s='A trim of the airplane for several combinations of V and γ where in the range where:', x=0.5, y=0.85, fontsize=12, ha='center', va='center')
    fig.text(s='16° ≤ α ≤ 12°      T ≥ 0 N      -20°≤ $δ_{E}$ ≤20°', x=0.5, y=0.82, fontsize=12, ha='center', va='center')

    x_labels = ["velocity, v [m$\mathregular{s^{-1}}$]",
               "flight Path Angle, γ [°]"]
        
    y_labels = ["thrust, T [N]",
                "elevator Angle, δE [°]"]

    for i in range(2):
            for j in range(2):
                ax[i, j].set_ylabel(y_labels[i],)
                ax[i, j].set_xlabel(x_labels[j])
                ax[i, j].grid(linestyle='-', color='black')
                ax[i,j].legend() # Must be called afer data is ploted otherwise legend isnt updated
    
    plt.show()

#-----------------------------------------------    B2    ----------------------------------------------------------------------------

class InclineSimulation (Simulation):
    def __init__(self,TrimV,TrimGamma,TotalSimulationTime,initial_h = 1000,final_h = 2000):
        """
        Finds the ideal climb time to get to a specified height after a 2 degree increase
        """
        Trim = Trim_Simulation(TrimV,TrimGamma)
        self.Trim = Trim

        gamma2 = TrimGamma + 2 * math.pi/180
        self.Trim2 = Trim_Simulation(TrimV,gamma2)
        
        Precision = 1000
        self.T_climb = Precision #Will imediately be reset back to 0

        
        while Precision >0.01:
            #Repeats until the required precision of time is met
            current_h = 0
            self.T_climb -= Precision # subtracts the last increment added so that we are below final height again
            Precision *= 0.1 # gets more precise
            while current_h < final_h: 
                # Finds the t_climb at which the final height is greater than 2000 
                # by going in increments of "precision"
                self.T_climb += Precision
                y = integrate.solve_ivp(self.Model,[0,TotalSimulationTime],[Trim.u_b,Trim.w_b,Trim.theta,0,0,0],t_eval=np.linspace(0,TotalSimulationTime,TotalSimulationTime*10))
                current_h = y.y[5][len(y.y[5]) - 1] * -1 + initial_h
                print(current_h,self.T_climb)
        
        print(f"Optimimum T_Climb: {self.T_climb}")
        print(f"Final Height: {current_h}")
        self.HandleSimulationData(y,1000)
        self.PlotData()
    

    def Model(self,t,y):
        # Only changes to trim 2 commands while in t_climb
        if t > 10 and t < 10 + self.T_climb:
            delta = self.Trim2.delta
            Thrust = self.Trim2.T
        else:
            delta = self.Trim.delta
            Thrust = self.Trim.T
            
        return self.Calculations(t,y,delta,Thrust)

    def SetTitles(self, fig):
        plt.subplots_adjust(top=0.75,bottom=0.075)
        fig.text(s="B2: Climb Analysis", x=0.5, y=0.9, fontsize=20, ha='center', va='center')
        fig.text(s='Analysis of flight altitude from h1 = 1000 m to h2 = 2000m by 3 trim conditions', x=0.5, y=0.87, fontsize=12, ha='center', va='center')
        fig.text(s="Trim condition 1: h1 = 1000 m (constant), V = 100 + 19, γ=0°", x=0.25, y=0.84, fontsize=12, ha='left', va='center')
        fig.text(s="Trim condition 2: 0m 1000 ≤ h2 ≤ 2000 m (increasing), V = 100 + 19, γ=2°", x=0.25, y=0.81, fontsize=12, ha='left', va='center')
        fig.text(s="Trim condition 3: h2 = 2000 m, V = 100 + 19, γ=0°", x=0.25, y=0.78, fontsize=12, ha='left', va='center')
        return fig
#-----------------------------------------------    Testing    ----------------------------------------------------------------------- 

# A1
def PrintCoefficients():
    # print coefficients into a pandas dataframe
    data = {
        'Coefficient': ['CL_0', 'C_L_alpha', 'C_L_Delta', 'C_D_0', 'C_D_k', 'C_M_0', 'C_M_alpha', 'C_M_Delta'],
        'Value': [CL_0, CL_alpha, CL_delta, CD_0, CD_k, CM_0, CM_alpha, CM_delta]
    }

    df = pd.DataFrame(data)
    df_title = pd.DataFrame({'Coefficient': [''], 'Value': ['']})
    df = pd.concat([df_title, df], ignore_index=True)

    print(df.to_markdown(index=False, tablefmt='grid'))

    fig, ax = plt.subplots()
    ax.text(0.5, 0.5, df.to_string(index=False),
            ha='center', va='center', fontsize=14, transform=ax.transAxes)
    ax.axis('off')
    plt.show()

# A2
def Test_Trim(v,gamma):
    sim = Trim_Simulation(v,gamma)
    print(f"Alpha: {sim.alpha}")
    print(f"Delta: {sim.delta}")
    print(f"Thrust: {sim.T}")
    print(f"Theta: {sim.theta}")
    print(f"u_b: {sim.u_b}")
    print(f"w_b: {sim.w_b}")

# A3
CustomSimulation(100,0,300,10,100)

# B1
B1()

# B2
InclineSimulation(119,0,600)
