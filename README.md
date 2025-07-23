
# UAV NMPC Controller

This repository contains the MATLAB implementation of a Nonlinear Model Predictive Control (NMPC) strategy for a tilting-propeller quadrotor (Voliro-style UAV). The drone navigates through a 3D environment with transparent walls and ceiling to improve animation visibility.

## Features

- Nonlinear MPC controller for UAV with 8 control inputs (4 thrusts, 4 tilting angles)
- 3D reference trajectory tracking with hover phase
- Simulation with ceiling effect and tiltable propellers
- Continuous dynamic model implemented via custom MATLAB function
- Integrated in Simulink with NMPC block

## Files

- `main.m`: Script that defines NMPC object, model, reference trajectory and starts simulation.
- `UAV_model.m`: Function implementing the drone's nonlinear dynamics.
- `NMPC_ane.slx`: Simulink model with the NMPC controller.
- `README.md`: This file.

## Simulation Scenario

- A 3D scene with two rooms and a target point at `(9, 9, 0.5)`.
- South and east walls and the ceiling are transparent to facilitate animation viewing.
- The drone hovers for 3 seconds at the initial position before starting the mission.

## Requirements

- MATLAB R2024b or newer
- Model Predictive Control Toolbox
- Simulink

## Usage

1. Open `main.m` and run it to initialize variables and start the simulation.
2. Open `NMPC_ane.slx` to inspect or edit the control structure.
3. Customize the reference trajectory or model as needed.

---

© 2025 – UAV NMPC Simulation Project
