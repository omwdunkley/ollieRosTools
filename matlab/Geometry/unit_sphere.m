function [ rays ] = unit_sphere(CAM, f_img )
%UNIT_SPHERE Given a vector of 2d features, project them to a unit sphere

    % cam -> world (2d lines)   DIDNT WORK??  
    f_img = [f_img(:,1:2)'; ones(1,size(f_img,1))];
    
    % extract K from cam
    %CAMinv = inv(CAM(1:3,1:3));    
    %rays = CAMinv * f_img;    
    % Or in matlab style
    rays = CAM(1:3,1:3)\f_img;
    
    % make unit length
    rays = unit_vector(rays);
    
    % add ones
    rays = [rays; ones(1,size(rays,2))]; 


end

