
%% Plot movie


f=figure(1);
% set(f,'position',[FX FY FX FY],'units','pixels')
% set(gca, 'Position', get(gca, 'OuterPosition') - get(gca, 'TightInset') * [-1 0 1 0; 0 -1 0 1; 0 0 1 0; 0 0 0 1]);
set(gca,'position',[0 0 1 1],'units','normalized')
set(gcf,'position',[CX*2 CY*2 CX*2 CY*2],'units','pixels')
axis off
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
    

    axes1 = axes('Parent',f,...
    'YTick',[0:50:(CY*2)-1 CY*2],...
    'YMinorTick','on',...
    'YDir','reverse',...
    'XTick',[0:50:(CX*2)-1 CX*2],...
    'XMinorTick','on',...
    'Units','pixels',...
    'Position',[0 0 CX CY].*2,...
    'PlotBoxAspectRatio',[CX CY 1],...
    'DataAspectRatio',[1 1 1]);

     xlim(axes1,[0 CX*2]);
     ylim(axes1,[0 CY*2]);
    grid(axes1,'on');
    hold(axes1,'all');
    
    hold on;
    % features, colors based on distance    
    quiver(f2d_pre(:,1),f2d_pre(:,2),f2d_predif(:,1),f2d_predif(:,2),0,'m');   
    scatter(f2d(:,1), f2d(:,2), 30, dm);     
    grid on
    hold off;
    
    %% SAVE IMAGE
    if SAVE
        fra = getframe(gcf); 
        imwrite(fra.cdata,sprintf([PATH '/images/img%03d.png'],step));    

        %% SAVE FEATURES 2d
        camFeatId = [find(in_view) f2d];
        csvwrite(sprintf([PATH '/features/f2d%03d.csv'],step), camFeatId)
    else
       drawnow; 
    end   
    
end

if SAVE
    %% Saves poses
    %% x y t qx qy qz qw

    RCs = W2C(1:3,1:3,2:end);
    TCs = W2C(1:3,4,2:end);
    QCs = dcm2quat(RCs);
    TCAM =  [squeeze(TCs)' QCs];
    csvwrite(sprintf([PATH '/trajectory/TCAM.csv']), TCAM)

    % ROs = W2OBJ(1:3,1:3,2:end);
    % TOs = W2OBJ(1:3,4,2:end);
    % QOs = dcm2quat(ROs);
    % TOBJ =  [squeeze(TOs)' QOs];
    csvwrite(sprintf([PATH '/trajectory/TOBJ.csv']), tp(2:end,:))






    %% Save 3d cloud
    cloud = [[2:nr_f]' features(1:3,2:end)'];
    csvwrite([PATH '/points3d/cloud.csv'], cloud)

    %% Save transforms
    csvwrite([PATH '/meta/cam2img.csv'], CAM2IMG)


    Rimu2cam = IMU2CAM(1:3,1:3);
    Timu2cam = IMU2CAM(1:3,4);
    Qimu2cam = dcm2quat(Rimu2cam);
    TIMU2CAM =  [Timu2cam' Qimu2cam];
    csvwrite([PATH '/meta/imu2cam.csv'], TIMU2CAM)
    save([PATH '/meta/workspace']);
end

