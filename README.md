# Quadcopter-stability-MPC

## Abstract
The aim of this report is to implement a Multi-Input Multi-Output controller to
a plant model. The plant model used is a Quadrotor, the parameters of which has
been derived from DJI Phantom 2. The goal of the quadrotor is to align with the
reference trajectory input by the user. A non-linear model predictive control has been
implemented on the quadrotor using MATLAB Model Predictive Control Toolbox.
Wind disturbance is introduced and evaluated in various cases. Parameter tuning is
done using a grid search method. The paper also evaluated the effectiveness of MPC
as a controller on a Quadrotor

## Usage

Open file NLMPC_demo.m and run it using MATLAB

### Try

Change Vx and Vy for different use cases
