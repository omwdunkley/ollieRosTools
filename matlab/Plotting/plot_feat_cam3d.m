%% Cam view 3d
figure(103); clf;
set(gca, 'LooseInset', get(gca,'TightInset'))
view(-19,52)

% limits = [min(min(features_cam(1,:,:))) max(max(features_cam(1,:,:))) ...
%           min(min(features_cam(2,:,:))) max(max(features_cam(2,:,:))) ...
%           min(min(features_cam(3,:,:))) max(1,max(max(features_cam(3,:,:))))];
limits = [-8 8 -8 8 -8 8];
for step=1:nr_t

    plot3(features_cam(:,1,step),features_cam(:,2,step),features_cam(:,3,step), 'o','Color',[0.8 0.8 0.8]);
    
    hold on;
    % plot visible ones
    in_view = logical(features_img(:,3,step));
    dists = feature_dists(in_view, step);

    scatter3(features_cam(1,in_view,step)',features_cam(2,in_view,step)',features_cam(3,in_view,step)', 50,depth_map(dists,1,15),'*');
%     scatter3(features_cam(in_view,1,step),features_cam(in_view,2,step),features_cam(in_view,3,step), 40,depth_map(dists,0,0));
    

%     plot_cam(CAM,[0 0 0], [1 0 0 0])
    plot_frame(eye(4),0.5, 'CAM')
%     
%     %rays = unit_sphere(CAM, features_img(in_view,1:2, step));
%     rays = features_img_sphere(in_view,:,step);
%     
%     
%     
%      for r = 1:size(rays,1)
%            plot3([0 rays(r,1)], ...
%                  [0 rays(r,2)], ...
%                  [0 rays(r,3)], 'k','LineWidth',0.1);           
%            plot3([0 rays(r,1)*dists(r)], ...
%                  [0 rays(r,2)*dists(r)], ...
%                  [0 rays(r,3)*dists(r)], ':b');
%      end

    % Plot Coord at 0,0,0
    CS = eye(3).*0.5; 
    plot3([0 CS(1,1)], [0 CS(1,2)], [0 CS(1,3)],'r','LineWidth',2.5);        
    plot3([0 CS(2,1)], [0 CS(2,2)], [0 CS(2,3)],'g','LineWidth',2.5);    
    plot3([0 CS(3,1)], [0 CS(3,2)], [0 CS(3,3)],'b','LineWidth',2.5); 
    hold off;
%     axis ij;
    axis equal;
    axis(limits);
    title( sprintf('3d Cam %d/%d', step,nr_t))
%     view(0,90)
    view(-19,52)
    grid on;
    xlabel('x')
    ylabel('y')
    zlabel('z')
    drawnow;    
end
