function [K, FX, FY, CX ,CY] = pinhole(width, height, horFovDeg, verFovdeg)
%% Returns a camera matrix given image width and heigh and horizontal/vertical FOV in degrees
FX = width*0.5/tan(deg2rad(horFovDeg/2));
CX = width/2;
FY = height*0.5/tan(deg2rad(verFovdeg/2));
CY = height/2;

K  =[FX 0  CX 0;... 
    0   FY CY 0; ...
    0   0  1  0];