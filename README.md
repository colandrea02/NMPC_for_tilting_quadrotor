
# UAV NMPC Controller

This repository contains the MATLAB implementation of a Nonlinear Model Predictive Control (NMPC) strategy for a tilting-quadrotor.

## Features

- Nonlinear MPC controller for UAV with 8 control inputs (4 thrusts, 4 tilting angles)
- 3D reference trajectory tracking with hover phase
- Simulation with ceiling and ground effect, and tiltable propellers
- Continuous dynamic model implemented via custom MATLAB function using the Voliro approach
- Integrated in Simulink with NMPC block
- 3D animation in an indoor environment

## Files

- `main_tilting.m`: Initializes simulation parameters, defines the NMPC controller and system dynamics.
- `plot_all.m`: Script that allows to plot all the interested variables.
- `NMPC.slx`: Simulink model implementing the NMPC-based control of the tilt-rotor UAV.
- `animation.m`: Creates a 3D animation of the UAV trajectory and motion.
- `ref_trajectory.m`: Defines waypoints and interpolates the reference trajectory.
- `quadrotor_3.stl`: STL model of the drone

## Simulation Scenario

- A 3D scene with two rooms and a target point at `[9.0, 9.0, 1.0]`.
- South and east walls and the ceiling are transparent to facilitate animation viewing.

## Requirements

- MATLAB R2024b or newer
- Model Predictive Control Toolbox
- Simulink

## Usage Instructions
- Open main_tilting.m and set the desired disturbance type by configuring the disturbance_type variable:
    - 0 → No disturbance
    - 1 → Constant wind-like disturbance on X and Y axes
    - 2 → Stochastic noise on all axes
- Run the script to start the simulation.
- After the simulation completes, execute plot_all.m to generate and display the plots of relevant variables.



