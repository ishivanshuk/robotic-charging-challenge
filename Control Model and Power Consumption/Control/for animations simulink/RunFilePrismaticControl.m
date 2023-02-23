clear;
clc;

%% Run Simulation
[Basic_arm_rbt, ArmInfo1] = importrobot('Basic_arm');

open_system ('PrismaticControl.slx');
sim('PrismaticControl.slx');