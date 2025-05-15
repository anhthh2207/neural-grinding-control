<!-- # PID Controller for Robot End-Effector

This section provides the Simulink model for simulating the compliant end-effector’s control system (refer to [paper](https://www.sciencedirect.com/science/article/pii/S0736584523001217)), along with the code used to fine-tune the PID controller that regulates the output force $F_0$ at the end effector. 

## Running intruction:
- Modify the parameters in the config.m file as needed, then execute the script. This will load the parameters into the workspace for use with the Simulink model.
- The controller is automatically tuned using MATLAB's built-in pidtune function. The optimized gain values are then extracted and applied to the Simulink model.
- Finally, run the simulation in Simulink to observe the system's response. -->

# PID Controller for Robot End-Effector

This section provides the Simulink model for simulating the compliant end-effector’s control system (refer to [paper](https://www.sciencedirect.com/science/article/pii/S0736584523001217)), along with the code used to fine-tune the PID controller that regulates the output force $F_0$ at the end effector. 

## 1. Trasfer function for compliant end-effector
 $$M\ddot{X}_0 - K_s(X_m - X_0) - C_f(X'_m - X'_0) + F_0 = 0$$
 $$ F_0 = K_e X_0 $$

From these 2 equations, the transfer funciton in Laplace domain
$$\frac{F_0(s)}{X_m(s)} = \frac{K_e C_fs + K_e K_s}{M s^2 + C_f s + K_e + K_s}$$


## 2. Usage
To run built-in MATLAB tuner
```matlab
pid_tuner
```

To run Genetic Algorithm (GA) tuner (less stable)
```matlab
pid_ga
```

To try Reinforcement Learning (RL) tuner, run the live scripts [rl_tuner.mlx](rl_tuner/rl_tuner.mlx).


## 3. Simulink model:
- Modify the parameters in the `config.m` file as needed, then execute the script. This will load the parameters into the workspace for use with the Simulink model.
- The controller is automatically tuned using MATLAB's built-in pidtune function. The optimized gain values are then extracted and applied to the Simulink model.
- Finally, run the simulation in Simulink to observe the system's response.

### TODO
- [ ] Finetune GA tuner (more interations, more samples, abitrary reference waveform)
- [ ] Finetune RL agent.
    - [ ] Larger networks
    - [ ] Dynamics learning rate
    - [ ] Balance exploration-exploitation
