clear;
clc;
%% Run Prismatic Joint based control model

%model might take time to run due to simulation
open_system ('Prismatic_Joint_Control.slx');
sim('Prismatic_Joint_Control.slx');

%% Trajectory

figure;
plot( ans.x  );
title('x vs t');

figure;
plot( ans.y  );
title('y vs t');

figure;
plot( ans.z  );
title('z vs t');

figure;
plot3( ans.x, ans.y, ans.z );
title('Trajectory');