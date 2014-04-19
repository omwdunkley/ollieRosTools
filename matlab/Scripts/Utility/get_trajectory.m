function [ pos, vel, acc ] = get_trajectory(NR,ID)
%GET_TRAJECTORY Returns a trajectory pos, vel, acc

%% Params
% NR = 120; % Nr of steps in trajectory


%% Compute poses. Rotation in camera frame

% Smooth continuous motion
if ID==1
    xyz = [-linspace(-4,4,NR)'...
        cos(4*linspace(-0.7,0.7,NR))'...
        cos(10*linspace(0.9,1.5,NR)')];
    xyz = [xyz(:,1), xyz(:,2), xyz(:,3)+1];
    rpy = [-deg2rad(5*cos(linspace(pi,4*pi,NR)).*linspace(0,15,NR))'...
        deg2rad(linspace(-10,20,NR))'...
        deg2rad(90+20*cos(linspace(pi,3*pi,NR)))'];
end


% ZigZag
if ID==2
    NR = ceil(NR/3);
    xyz = [ linspace(-4,0,NR)' linspace(3,3,NR)' linspace(0,0,NR)'];                % move right
    xyz = [xyz; [linspace(0,0,NR)' linspace(3,-1,NR)' linspace(0,0,NR)']];          % move forward
    xyz = [xyz; [linspace(0,0,NR)' linspace(-1,-1,NR)' linspace(0,4,NR)']];         % move up
    rpy = deg2rad([linspace(0,0,NR*3)' linspace(0,0,NR*3)' 90+linspace(0,0,NR*3)']); % no rotation
end


% turn all directions
if ID==3
    NR = ceil(NR/3);
    xyz = [ linspace(0,0,NR*3)' linspace(-1,-1,NR*3)' linspace(1,1,NR*3)'];             % No motion
    rpy = [60*sin(linspace(0,2*pi,NR))' linspace(0,0,NR)' 90+linspace(0,0,NR)'];        % up down up roll_w pitch_c
    rpy = [rpy; [linspace(0,0,NR)' 60*sin(linspace(0,2*pi,NR))' 90+linspace(0,0,NR)']];  % left-right-left pitch_w roll_c
    rpy = [rpy; [linspace(0,0,NR)' linspace(0,0,NR)'  90+60*sin(linspace(0,2*pi,NR))']]; % left-right-left yaw_w yaw_c
    rpy = deg2rad(rpy);
end

% turn all directions at same time
if ID==4
    %xyz = [ linspace(0,0,NR)' linspace(-1,-1,NR)' linspace(1,1,NR)'];             % No motion
    xyz = [ linspace(-2,2,NR)' linspace(-1,1,NR)' linspace(-1,1,NR)']; 
    rpy = [30*sin(linspace(0,2*pi,NR))' 30*sin(linspace(0,2*pi,NR))' 90+30*sin(linspace(0,2*pi,NR))'];        % up down up roll_w pitch_c
    rpy = deg2rad(rpy);
end




quat = angle2quat(rpy(:,1), rpy(:,2),rpy(:,3),'XYZ');

%% Compute velocities

xyzv = [diff(xyz); [ 0 0 0]];
rpyv = [diff(rpy); [ 0 0 0]];
quatv = angle2quat(rpyv(:,1), rpyv(:,2),rpyv(:,3),'XYZ');

%% Compute accelerations
xyza = [diff(xyzv); [ 0 0 0]];
rpya = [diff(rpyv); [ 0 0 0]];
quata = angle2quat(rpya(:,1), rpya(:,2),rpya(:,3),'XYZ');

%% Assemble trajectories
pos = [ xyz  quat ];
vel = [ xyzv quatv];
acc = [ xyza quata];
 

 end

