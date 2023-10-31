# -----------------------------------------------
# Input for initial trim
# Trim velocity [m/s]
TrimVelocity = 100 # e.g. 100 (m/s)
# Trim flight path angle [radian]
TrimGamma = 0.05 # e.g. 0.05 (rad)
# -----------------------------------------------
# -----------------------------------------------
# Input for step change of commands
# Total simulation time (seconds)
TotalTimeSimulation = 200 # e.g. 200 (seconds)
# Increase elevator angle by
# a percentage PercentageChangeElevator
# at a certain time TimeChangeElevator
PercentageChangeElevator = 10 # e.g. 10 (\%)
TimeChangeElevator = 20 # e.g. 20 (seconds)
# Increase Thrust by
# a percentage PercentageChangeThrust
# at a certain time TimeChangeThrust
PercentageChangeThrust = 10 # e.g. 10 (\%)
TimeChangeThrust = 30 # e.g. 30 (seconds)
# -----------------------------------------------

from aero_analytical_build_ToBeCompleted import CustomSimulation

CustomSimulation(TrimVelocity,TrimGamma,TotalTimeSimulation,PercentageChangeElevator,TimeChangeElevator,PercentageChangeThrust,TimeChangeThrust)
