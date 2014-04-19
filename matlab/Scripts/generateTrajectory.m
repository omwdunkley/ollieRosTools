clc; clear all; 


%% PARAMS
generateTrajectorySettings


%% Get points
features = get_features(99, NR_F);
nr_f = size(features,2);

%% Get Traj
[tp, tv, ta] = get_trajectory(NR_T, TRAJ_ID);
nr_t = size(tp,1);

%% Convert to matrix form
W2IMU = tq2t(tp); % position cam ref frame
CRv = tq2t(tv); % velocity
CRa = tq2t(ta); % accel
W2C = t_cons(W2IMU,IMU2CAM);  % position cam frame




%% Get Features in camera frames along trajectory (3d)
features_cam    = zeros(4, nr_f, nr_t);
% features_camref = zeros(4, nr_f, nr_t);
for step=1:nr_t
    
    % Feat in camref
%     features_camref(:,:,step) = t_apply(t_inv(W2CR(:,:,step)), features);
    
    % Camref to cam
%     features_cam(:,:,step) = t_apply(CAMREF2CAM, features_camref(:,:,step));  

    features_cam(:,:,step) = t_apply(t_inv(W2C(:,:,step)), features);
    
end

%% Distance from every step to every feature
feature_dists = zeros(nr_f,nr_t);
for step=1:nr_t
    feature_dists(:,step) = sqrt(sum(features_cam(1:3,:,step) .* features_cam(1:3,:,step),1))';                     
end





%% Project Features on camera plane along trajectory (2d)
features_img = zeros(nr_f,3,nr_t);
for step=1:nr_t
    % make homo and project
    f2d = CAM2IMG*features_cam(:,:,step);
    % keep homo values 1
    f2d = bsxfun(@times,f2d,1./f2d(3,:));    
    % Calculate bounds
    idx = f2d(1,:)<=CX*2 & f2d(1,:)>=0 & f2d(2,:)<=CY*2 & f2d(2,:)>=0;% & features_cam(3,:,step)'<0 ;    
    % Save. Last column is true if in field of view
    features_img(:,:,step) = f2d';    
    features_img(~idx,3,step) = 0;
end




%% Project Features onto unit sphere
features_img_sphere = zeros(4,nr_f,nr_t);
for step=1:nr_t
    features_img_sphere(:,:,step) = unit_sphere(CAM2IMG, features_img(:,1:2,step));
end




% Plot Scene
% plot_scene

% Plot Movie 2d-3d
% plot_movie

% Plot 3d features from cam view
% plot_feat_cam3d







% kfs = [1];
% gyro_integration = [0 0 0];
% disparity_limit = 0.3;
% disparities = zeros(nr_t,1);
% gyro = [1 0 0 0];
% 
% for frame = 1:nr_t    
%     
%     frame1 = kfs(end); t1 = tp(frame1, 1:3); q1 = tp(frame1, 4:7);    
%     frame2 = frame; t2 = tp(frame2, 1:3); q2 = tp(frame2, 4:7);
%     gyro = quatmultiply(gyro, tv(frame2, 4:7));
%         
%     % Get idx of features visible in both frames
%     f1_vis = logical(features_img(:,3,frame1));
%     f2_vis = logical(features_img(:,3,frame2));
%     f1f2_vis = f1_vis & f2_vis;
% 
%     % Get corresponding 2d features
%     f1 = features_img(f1f2_vis,1:2,frame1);
%     f2 = features_img(f1f2_vis,1:2,frame2);
% 
%     % Compute disparity
%     disparity = f2f_disparity(f1,f2);
%     disparities(frame) = disparity;
% 
%     % Update keyframe if disparity large enough
%     if disparity>disparity_limit
%        kfs = [ kfs frame];
%        % reset rotation prior
%        [R, P, Y] = quat2angle(gyro, 'xyz');
%        gyro_integration = [gyro_integration; [R, P, Y]];
%        gyro = [1 0 0 0];       
%     end
%     
%     
%     %continue
%     
%     %% PLOT
% 
%     figure(105); clf; set(gca, 'LooseInset', get(gca,'TightInset'))
%     limits = [min(min(features(:,1))) max(max(features(:,1))) ...
%               min(min(features(:,2))) max(1,max(max(features(:,2)))) ...
%               min(min(features(:,3))) max(max(features(:,3)))];
%           
%           
%           
%           
%           
%           
% 
%     % Plot Coord at 0,0,0
%     CS = eye(3).*0.5; 
%     plot3([0 CS(1,1)], [0 CS(1,2)], [0 CS(1,3)],'r','LineWidth',2.5); 
%     hold on
%     plot3([0 CS(2,1)], [0 CS(2,2)], [0 CS(2,3)],'g','LineWidth',2.5);    
%     plot3([0 CS(3,1)], [0 CS(3,2)], [0 CS(3,3)],'b','LineWidth',2.5); 
% 
% 
%     % Plot trajecotry Lines
%     plot3(tp(:,1), tp(:,2), tp(:,3),'m');
% 
%     % Plot Start
%     text(tp(1,1), tp(1,2), tp(1,3),'Start');    
% 
%     % plot all features in grey
%     plot3(features(:,1),features(:,2),features(:,3), 'o','Color',[0.8 0.8 0.8]);
% 
% 
%     axis ij;
%     axis equal;
%     axis(limits);
%     xlabel('x')
%     ylabel('y')
%     zlabel('z')
%     % title( sprintf('3d Cam %d/%d', step,nr_t))
%     grid on;   
% 
% 
% 
% 
%     text(t1(1),t1(2),t1(3),num2str(frame1));
%     text(t2(1),t2(2),t2(3),num2str(frame2));
% 
%     % Plot key frames so far
%     for d = kfs
%         plot_cam(CAM,tp(d, 1:3),tp(d, 4:7));
%         text(tp(d, 1),tp(d, 2),tp(d, 3),num2str(d));
%     end
%     % Plot current key frame and frame
%     plot_cam_rays(CAM, t1,q1, features_cam(:,:,frame1), features_img(:,:,frame1), features,'--b' )
%     plot_cam_rays(CAM, t2,q2, features_cam(:,:,frame2), features_img(:,:,frame2), features, '--m' )
%     hold off;
% 
% 
%     
%     
%     figure(107);
%     plot(disparities)
%     grid on
%     xlabel('Step')
%     ylabel('Median 2d feature disparity')
%     text(frame,disparities(frame), sprintf('%d = %2.3f', frame, disparities(frame)));
%     axis([0 frame 0 disparity_limit*1.2])
%     
%     
%     % Draw camera view
%     figure(106); clf; set(gca, 'LooseInset', get(gca,'TightInset'))
%     fdiff = f2-f1;
%     quiver(f1(:,1), f1(:,2), fdiff(:,1), fdiff(:,2),0);
%     axis ij;
%     axis equal;
%     axis([-0.5 0.5 -0.5 0.5])
%     xlabel('x')
%     ylabel('y')    
%     title(sprintf('Frame %d vs Key-Frame %d', frame2, frame1))
% 
%     drawnow;
%     %%
% end

% 
% 
% %%  4.2.3 Relative pose
% % test getting poses between key frames
% 
% from = 1;
% to = 2;
% kf1 = kfs(from);
% kf2 = kfs(to);
% 
% gyro = gyro_integration(2,:); % rpy






