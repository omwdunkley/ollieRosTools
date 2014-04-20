%% Testing Relative Rotation

KF = 3;
F  = 30;

% Poses CAM
c1 = W2C(1:3,1:3,KF);
c2 = W2C(1:3,1:3,F);

W2C(:,:,KF)
W2C(:,:,F)

% Poses IMU
i1=W2IMU(1:3,1:3,KF);
i2=W2IMU(1:3,1:3,F);

% Bearings
bv1 = squeeze(features_img_sphere(1:3,:,KF));
bv2 = squeeze(features_img_sphere(1:3,:,F));

i2c=IMU2CAM(1:3,1:3);



figure(1); clf; 
subplot(2,1,1);
o = zeros(1,size(bv2,2));
line([bv1(1,:); o], [bv1(2,:); o], [bv1(3,:);o],'color','b')
line([bv2(1,:); o], [bv2(2,:); o], [bv2(3,:);o],'color','g')
line([bv1(1,:); bv2(1,:)], [bv1(2,:); bv2(2,:)], [bv1(3,:); bv2(3,:)],'color','r')
axis equal; axis([-1 1 -1 1 -1 1])
grid on;


% Angle between them
e = zeros(size(bv1,2),1);
for i = 1:size(bv1,2)
    e(i) = rad2deg(atan2(norm(cross(bv1(:,i), bv2(:,i))), dot(bv1(:,i), bv2(:,i))));
end
mean(e)






%% Unrotate
R = i2c' * i1' * i2 * i2c; 
bv2 = (bv2' * R');
bv2 = bv2';

 
subplot(2,1,2);
o = zeros(1,size(bv2,2));
line([bv1(1,:); o], [bv1(2,:); o], [bv1(3,:);o],'color','b')
line([bv2(1,:); o], [bv2(2,:); o], [bv2(3,:);o],'color','g')
line([bv1(1,:); bv2(1,:)], [bv1(2,:); bv2(2,:)], [bv1(3,:); bv2(3,:)],'color','r')
axis equal; axis([-1 1 -1 1 -1 1])
grid on;


% Angle between them
e = zeros(size(bv1,2),1);
for i = 1:size(bv1,2)
    e(i) = rad2deg(atan2(norm(cross(bv1(:,i), bv2(:,i))), dot(bv1(:,i), bv2(:,i))));
end
mean(e)


