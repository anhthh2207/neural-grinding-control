# PID Controller for Robot End-Effector

This section provides the Simulink model for simulating the compliant end-effector’s control system (refer to [paper](https://www.sciencedirect.com/science/article/pii/S0736584523001217)), along with the code used to fine-tune the PID controller that regulates the output force $F_0$ at the end effector. 

## Running intruction:
- Modify the parameters in the config.m file as needed, then execute the script. This will load the parameters into the workspace for use with the Simulink model.
- The controller is automatically tuned using MATLAB's built-in pidtune function. The optimized gain values are then extracted and applied to the Simulink model.
- Finally, run the simulation in Simulink to observe the system's response.
