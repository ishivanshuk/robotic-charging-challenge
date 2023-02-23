clear;
clc;

%% Run Simulation
[Basic_arm_rbt, ArmInfo1] = importrobot('Basic_arm');

open_system ('ManipulatorControl.slx');
sim('ManipulatorControl.slx');