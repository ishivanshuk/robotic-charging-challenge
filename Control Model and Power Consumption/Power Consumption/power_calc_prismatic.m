%% Power calculation for prismatic joint approach

%model might take time to run due to simulation
clear;
clc;
open_system ('Prismatic_Power_Simulink.slx');
sim('Prismatic_Power_Simulink.slx');

%% Calculation
p_azimuth = ans.power_azimuth;
p_elevation = ans.power_elevation;

p_actuator = ans.power_actuator;

tp1 = sum(p_azimuth, 'all')/1000; 
tp2 = sum(p_elevation, 'all')/1000;
tp3 = sum(p_actuator, 'all')/1000;

len = size(p_actuator);

total_power_consumption = 2*(tp1 + tp2 + tp3) + 30*( p_actuator(len) + p_azimuth(len) + p_elevation(len) );

total_power_consumption = total_power_consumption/1000;

fprintf('Total power consumption (kJ) = %f\n ', total_power_consumption(1));

