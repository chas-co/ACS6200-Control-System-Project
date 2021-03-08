# ACS6200-MSc Control System Project

## Project Brief
Power electronic converters are inherently nonlinear and the auxiliary systems supply require stable and efficient power supply. Additionally, parallel connected power electronic converters must share the load current equally to minimise the stress on each converter. In this study, the state space averaging method was used to derive a linear time invariant (LTI) model of the standalone buck converter and two paralleled buck converters. The linear quadratic integral (LQI) controller and the linear quadratic regulator (LQR) based PID controller were proposed for the single buck converter. Additionally, The LQI controller and a two-level droop control method were designed for two paralleled buck converters. The controller designs were verified through simulations with circuit models of the buck converter operating in continuous conduction mode (CCM). The simulations were done with Simscape in the MATLAB/Simulink environment.

## Dependencis for Running Locally
* MATLAB
* Simulink
* Simscape Electrical

## Basic run instructions 
1. Run m file to load variables into the workspace
2. Open corresponding slx file and run the program
3. Open scope to observe the waveforms of the states and the output.