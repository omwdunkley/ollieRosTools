
%% Plot movie
figure(102); clf; set(gca, 'LooseInset', get(gca,'TightInset'));
for step=2:nr_t  
    
    

    %% 2D Stuff
    % Features in view (2d, 3dc, 3dw)
    in_view = logical(features_img(:,3,step));    
    % 2d Features Current
    f2d =    features_img(in_view,1:2,step);
    % 2d Features Previous
    f2d_pre = features_img(in_view,1:2,step-1);
    % Flow of each feature from pre to current
    f2d_predif = f2d-f2d_pre;
    % Distance to each feature from the cam
    dists = feature_dists(in_view,step);
    dm = depth_map(dists,1,10);
    
    %% 2D PLOT
    % 2d camera with optical flow
    subplottight(1,2,1); cla; hold on;    
    % optical flow
    quiver(f2d_pre(:,1),f2d_pre(:,2),f2d_predif(:,1),f2d_predif(:,2),0,'m');   
    % features, colors based on distance    
    scatter(f2d(:,1), f2d(:,2), 30, dm);    
    axis ij;
    axis equal;
    grid on;
    axis([0 720 0 480]);
    title( sprintf('2d Cam Frame %d/%d', step,nr_t))
  
    
    %% 3d Stuff
    % Get unit sphere feature projection
%     rays = unit_sphere(CAM2IMG, f2d);
    unit_rays = features_img_sphere(:,in_view,step);   
    unit_rays_w = t_apply(W2C(:,:,step),unit_rays);
    % Multiply by distance
    rays = unit_rays .* [dists dists dists ones(size(dists,1),1)]';    
    rays_w = t_apply(W2C(:,:,step),rays);
    
  
    
    
    
    
    %% 3d Plot
    % 3d scene
    subplottight(1,2,2); cla; hold on;
    % Plot Start
    text(tp(1,1), tp(1,2), tp(1,3),'    Start');   
    % all features
    plot3(features(1,:)', features(2,:)', features(3,:)','o','Color',[0.7 0.7 0.7]);
    % Visible features with color
    scatter3(features(1,in_view),features(2,in_view),features(3,in_view), 40,dm,'*');    
    % Plot Coord at 0,0,0
    plot_frame(eye(4), 0.5, ' World');    
    % Plot trajecotry Lines
    plot3(tp(:,1), tp(:,2), tp(:,3),'--m');
    % Plot current camera (ref and cam frame)
    plot_frame(W2C(:,:,step), 0.5, sprintf(' %d',step));
    plot_frame(W2IMU(:,:,step), 0.15, sprintf(' CF'));
    % Plot translation between refcam and cam frame
    trajcam_xyz = t_translation(W2C(:,:,step));
    trajcamref_xyz = t_translation(W2IMU(:,:,step));
    plot3([trajcam_xyz(1) trajcamref_xyz(1)], ...
          [trajcam_xyz(2) trajcamref_xyz(2)], ...
          [trajcam_xyz(3) trajcamref_xyz(3)], ':m' );  
      
    

    for r=1:size(rays_w,2)        
        plot3([trajcam_xyz(1) rays_w(1,r)],[trajcam_xyz(2) rays_w(2,r)],[trajcam_xyz(3) rays_w(3,r) ],':b')
        plot3([trajcam_xyz(1) unit_rays_w(1,r)],[trajcam_xyz(2) unit_rays_w(2,r)],[trajcam_xyz(3) unit_rays_w(3,r) ],'b')
    end
      
      
    title('Traj and features')
    xlabel('x')
    ylabel('y')
    zlabel('z')
    axis equal;
    grid on;
    drawnow;
    
    
    
    
end