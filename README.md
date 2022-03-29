# uwForm_VehicleDynamics


This folder contains only the main MATLAB scripts that run a 3-DOF vehicle model simulation. The files 'main.m' and 'main_control.m' correspond to the stand alone vehicle model and the vehicle model with a longitudinal brake control system implementation, respectively. These files serve to showcase the general structure of the vehicle model and the longitudinal controller. File dependencies are not included to protect potentially sensitive information surrounding the UW Formula team's race car. 

The 'main.m' simulation environment was built specifically for validating and tuning of the longitudinal controller as can be seen within the 'main_control.m' file, which is a reconfiguration of 'main.m'. However, 'main.m' can be used as stand alone although the user is tasked with developing their own source or model for the vehicle model inputs, throttle torque, brake torque, and steering angle. 
