%% load refs calculated using inverse kinematics
clear;
clc;
T = readmatrix('output.csv');

joint1 = timeseries( T(:, 2),   T(:, 1) ); 
joint2 = timeseries( T(:, 3),   T(:, 1) ); 
joint3 = timeseries( T(:, 4),   T(:, 1) ); 
joint4 = timeseries( T(:, 5),   T(:, 1) ); 

%% Power calculation for manipulator approach

%model might take time to run due to simulation
open_system ('Manipulator_Power_Simulink.slx');
sim('Manipulator_Power_Simulink.slx');

%% Calculation

p_joint1 = ans.joint1power;
p_joint2 = ans.joint2power;
p_joint3 = ans.joint3power;
p_joint4 = ans.joint4power;

tp1 = sum(p_joint1, 'all')/1000; 
tp2 = sum(p_joint2, 'all')/1000;
tp3 = sum(p_joint3, 'all')/1000;
tp4 = sum(p_joint4, 'all')/1000;

len = size(p_joint1);

total_power_consumption = 2*(tp1 + tp2 + tp3 + tp4) + 30*( p_joint1(len) + p_joint2(len) + p_joint3(len) + p_joint4(len ));

total_power_consumption = total_power_consumption/1000;

fprintf('Total power consumption (kJ) = %f\n ', total_power_consumption(1));
