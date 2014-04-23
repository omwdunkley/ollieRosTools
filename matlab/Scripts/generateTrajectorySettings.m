ZERO2ZERO = eye(4);
WORLD = eye(4);
[CAM2IMG, FX, FY, CX ,CY] = pinhole(720, 480, 110, 90);
%IMU2CAM = t_build([0 0 0],deg2rad([90 0 90]));
IMU2CAM = t_build([0.2 0 -0.1],deg2rad([90 0 90]));


SAVE = 1;
PATH = '/media/Evo/Dropbox/Code/ROS/ollieRosTools/matlab/Data/exportedHeart';
   
      
% Nr of features   
NR_F = 600;   

% Nr of trajectory steps
NR_T = 400;

% Trajectrory ID
TRAJ_ID = 5;
