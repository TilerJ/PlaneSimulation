import PySimpleGUI as sg
from aero_analytical_build import Trim_Simulation, CustomSimulation


# Create the GUI layout
layout = [
    [sg.Text("This code models the response of a small plane to different input conditions.")],
    [sg.Text("")],
    [sg.Text("Enter Velocity (m/s):"), sg.InputText(key="velocity")],
    [sg.Text("Enter Flight Path Angle (rad):"), sg.InputText(key="flight_angle")],
    [sg.Text("Angle of Attack (rad):"), sg.Text("", size=(20, 1), key="angle_of_attack")],
    [sg.Text("Thrust (N):"), sg.Text("", size=(20, 1), key="thrust")],
    [sg.Text("Elevator Angle (rad):"), sg.Text("", size=(20, 1), key="elevator_angle")],
    [sg.Button("Show Values")],
    [sg.Text("")],
    [sg.Text("Input for step change commands;")],
    [sg.Text("Total simulation time (s):"), sg.InputText(key="total_simulation_time")],
    [sg.Text("Change in Elevator Angle (%)"), sg.InputText(key="change_in_elevator_angle")],
    [sg.Text("Time at which the Elevator Angle changes (s):"), sg.InputText(key="elevator_angle_change_time")],
    [sg.Text("Change in Thrust (%)"), sg.InputText(key="change_in_Thrust")],
    [sg.Text("Time at which the Thrust changes (s):"), sg.InputText(key="Thrust_change_time")],
    [sg.Button("Show Graphs")],
]
    
# Create the PySimpleGUI window
window = sg.Window("User Interface", layout)

while True:
    event, values = window.read()

    if event == sg.WINDOW_CLOSED:
        break

    if event == "Show Values":
        TrimVelocity = float(values["velocity"])
        TrimGamma = float(values["flight_angle"])
        
        if TrimVelocity != None and TrimGamma != None: #Basic check to prevent errors
            trimconditions = Trim_Simulation(TrimVelocity, TrimGamma)
    
            angle_of_attack = trimconditions.alpha
            thrust = trimconditions.T
            elevator_angle = trimconditions.delta
    
            window["angle_of_attack"].update(f"{angle_of_attack:.4f} ")
            window["thrust"].update(f"{thrust:.4f}")
            window["elevator_angle"].update(f"{elevator_angle:.4f} ")
        
    if event == "Show Graphs":

        TrimVelocity = float(values["velocity"])
        TrimGamma = float(values["flight_angle"])
        TotalSimulationTime = int(values["total_simulation_time"])
        PercentageChangeElevator = float(values["change_in_elevator_angle"])
        TimeChangeElevator = float(values["elevator_angle_change_time"])
        PercentageChangeThrust = float(values["change_in_Thrust"])
        TimeChangeThrust = float(values["Thrust_change_time"])
        
        if TrimVelocity != None and TrimGamma != None:#Basic check to prevent errors
            CustomSimulation(TrimVelocity, TrimGamma, TotalSimulationTime,
                             PercentageChangeElevator, TimeChangeElevator,
                             PercentageChangeThrust, TimeChangeThrust, )

            
window.close()
