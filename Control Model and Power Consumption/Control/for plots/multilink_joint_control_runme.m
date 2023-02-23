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
open_system ('Multilink_Robot_Arm_control.slx');
sim('Multilink_Robot_Arm_control.slx');

%% Calculation

q1 = ans.joint1angle;
q2 = ans.joint2angle;
q3 = ans.joint3angle;
q4 = ans.joint4angle;

len = size(joint1);

X = zeros(3,len);

X(1,i) = (8*cos(q1(i))sin(q2(i)))/25 + (261*cos(q4(i))(cos(q1(i))cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)))/1000 + (261*sin(q4)(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)))/1000 + (7*cos(q1)*cos(q2)*sin(q3))/20 + (7*cos(q1)*cos(q3)*sin(q2))/20;

X(2,i) = (8*sin(q1(i))sin(q2(i)))/25 + (261*cos(q4(i))(cos(q2(i))sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)))/1000 - (261*sin(q4)(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)))/1000 + (7*cos(q2)*sin(q1)*sin(q3))/20 + (7*cos(q3)*sin(q1)*sin(q2))/20;

X(3,i) = (8*cos(q2(i)))/25 + (7*cos(q2(i))cos(q3(i)))/20 - (7*sin(q2(i))*sin(q3))/20 + (261*cos(q4)(cos(q2)cos(q3) - sin(q2)*sin(q3)))/1000 - (261*sin(q4)(cos(q2)*sin(q3) + cos(q3)*sin(q2)))/1000 + 87/100;

end
