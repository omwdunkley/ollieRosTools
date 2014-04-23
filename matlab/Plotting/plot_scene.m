%% Plot scene
figure(101); clf; set(gca, 'LooseInset', get(gca,'TightInset')); hold on;

% Plot Lines
trajcam_xyz = t_translation(W2C);
trajimu_xyz = t_translation(W2IMU);
plot3(trajimu_xyz(1,:), trajimu_xyz(2,:), trajimu_xyz(3,:),'m');

% Plot Start
text(trajimu_xyz(1,1), trajimu_xyz(2,1), trajimu_xyz(3,1),'Start');

plot_frame(WORLD, 1,'World')

% Plot Coords
for step=1:nr_t
%     plot_frame(W2OBJ(:,:,step),0.2,'CR');
    plot_frame(W2C(:,:,step),0.6,'C');
    plot_frame(W2IMU(:,:,step),0.2,'IMU');
    plot3([trajcam_xyz(1,step) trajimu_xyz(1,step)], ...
          [trajcam_xyz(2,step) trajimu_xyz(2,step)], ...
          [trajcam_xyz(3,step) trajimu_xyz(3,step)], ':m' );
end

% Features
depth_cmap = depth_map(features(2,:),0,0);
scatter3(features(1,:), features(2,:), features(3,:),40,depth_cmap);

% Setup
title('Traj and features')
xlabel('x')
ylabel('y')
zlabel('z')
% axis ij;
axis equal;
grid on;

% % Plot Camera locations
% for step=floor(linspace(1,nr_t,13))
%     q = tp(step,4:7);
%     xyz = tp(step,1:3);   
%     plot_cam( CAM, xyz, q )
% end

hold off 
