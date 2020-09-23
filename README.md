# Berthing-Simulation
Berthing simulation using Baxter manipulator arm

This code is dependent on the matlab-rigid-body-viz package: https://github.com/rpiRobotics/matlab-rigid-body-viz, the general-robotics-toolbox package: https://github.com/rpiRobotics/general-robotics-toolbox, and the subfunctions in the trajectory generation package: https://github.com/rpiRobotics/Space-Manipulator-Trajectory-Generation/tree/master/subfunctions. All packages should be added to the current path in Matlab while running. The required subfunctions must also be added to the path. The newest version of Matlab/Simulink may be required to run this simulation.

berthing_sim_example_run.m: This file is for running the berthing simulations. Different simulations can be chosen by commenting out/replacing the simulation model call in this example. 

notorque_sim.slx: Three berthing posts, force compliance only (i.e., no torque compliance in berthing algorithm).

threepost_sim.slx: Three berthing posts, full compliance algorithm (force and torque). 

fourpost_sim.slx: Four berthing posts, full compliance algorithm (force and torque). 
