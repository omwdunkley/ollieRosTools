%% Testing Relative Rotation

KF = 1;
FC = 60;
F  = 61;

% Poses CAM
cKF = W2C(1:3,1:3,KF)
cFC = W2C(1:3,1:3,FC)
c2 = W2C(1:3,1:3,F)

% Poses IMU
iKF=W2IMU(1:3,1:3,KF);
iFC=W2IMU(1:3,1:3,FC);
iF=W2IMU(1:3,1:3,F);

% Bearings
bvKF = squeeze(features_img_sphere(1:3,:,KF));
bvF = squeeze(features_img_sphere(1:3,:,F));

i2c=IMU2CAM(1:3,1:3);



figure(1); clf; 
subplot(2,1,1);
o = zeros(1,size(bvF,2));
line([bvKF(1,:); o], [bvKF(2,:); o], [bvKF(3,:);o],'color','b')
line([bvF(1,:); o], [bvF(2,:); o], [bvF(3,:);o],'color','g')
line([bvKF(1,:); bvF(1,:)], [bvKF(2,:); bvF(2,:)], [bvKF(3,:); bvF(3,:)],'color','r')
axis equal; axis([-1 1 -1 1 -1 1])
grid on;


% Angle between them
e = zeros(size(bvKF,2),1);
for i = 1:size(bvKF,2)
    e(i) = rad2deg(atan2(norm(cross(bvKF(:,i), bvF(:,i))), dot(bvKF(:,i), bvF(:,i))));
end
mean(e)





return
%% Unrotate use previous pose + relative imu

% KF vs CF pose



% CF vs F IMU
R = i2c' * iFC' * iF * i2c; 

R = R* cKF'*cFC;

bvF = (bvF' * R');
bvF = bvF';

 
subplot(2,1,2);
o = zeros(1,size(bvF,2));
line([bvKF(1,:); o], [bvKF(2,:); o], [bvKF(3,:);o],'color','b')
line([bvF(1,:); o], [bvF(2,:); o], [bvF(3,:);o],'color','g')
line([bvKF(1,:); bvF(1,:)], [bvKF(2,:); bvF(2,:)], [bvKF(3,:); bvF(3,:)],'color','r')
axis equal; axis([-1 1 -1 1 -1 1])
grid on;


% Angle between them
e = zeros(size(bvKF,2),1);
for i = 1:size(bvKF,2)
    e(i) = rad2deg(atan2(norm(cross(bvKF(:,i), bvF(:,i))), dot(bvKF(:,i), bvF(:,i))));
end
mean(e)




return
%% Unrotate use previous pose

R =  cKF' * cFC; 

bvF = (bvF' * R');
bvF = bvF';

 
subplot(2,1,2);
o = zeros(1,size(bvF,2));
line([bvKF(1,:); o], [bvKF(2,:); o], [bvKF(3,:);o],'color','b')
line([bvF(1,:); o], [bvF(2,:); o], [bvF(3,:);o],'color','g')
line([bvKF(1,:); bvF(1,:)], [bvKF(2,:); bvF(2,:)], [bvKF(3,:); bvF(3,:)],'color','r')
axis equal; axis([-1 1 -1 1 -1 1])
grid on;


% Angle between them
e = zeros(size(bvKF,2),1);
for i = 1:size(bvKF,2)
    e(i) = rad2deg(atan2(norm(cross(bvKF(:,i), bvF(:,i))), dot(bvKF(:,i), bvF(:,i))));
end
mean(e)

