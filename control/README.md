# PID Controller for Robot End-Effector

This section provides the Simulink model for simulating the compliant end-effector’s control system (refer to [paper](https://www.sciencedirect.com/science/article/pii/S0736584523001217)), along with the code used to fine-tune the PID controller that regulates the output force $F_0$ at the end effector. 

## Trasfer function for compliant end-effector
 $$M\ddot{X}_0 - K_s(X_m - X_0) + F_0 = 0$$
 $$ F_0 = K_e X_0 $$

From these 2 equations, the transfer funciton in Laplace domain
$$\frac{F_0(s)}{X_m(s)} = \frac{K_e K_s}{M s^2 + K_e + K_s}$$


## Running intruction:
- Modify the parameters in the config.m file as needed, then execute the script. This will load the parameters into the workspace for use with the Simulink model.
- The controller is automatically tuned using MATLAB's built-in pidtune function. The optimized gain values are then extracted and applied to the Simulink model.
- Finally, run the simulation in Simulink to observe the system's response.



### TODO
replace Ks, ke, M, Cf by emprical number

For optimized controller:
- split controller into seperated blocks: https://www.youtube.com/watch?v=N-pQUwWfa8Y

- more tutorial: https://www.youtube.com/watch?v=S5C_z1nVaSg&t=330s
