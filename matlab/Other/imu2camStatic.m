function [ output_args ] = imu2camStatic(rDownDeg)
x = 0.005;
y = 0;
z = -0.01;
r = -deg2rad(rDownDeg);
r=quatmultiply(angle2quat(-pi/2,0,-pi/2),angle2quat(0,0,r));

fprintf('rosrun tf static_transform_publisher %f %f %f  %f %f %f %f /cf0 /cam 50 # IMU->CAM\n', x,y,z,r(2),r(3),r(4),r(1))

end

