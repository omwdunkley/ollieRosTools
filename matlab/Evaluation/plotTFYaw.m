

bags = who('bag*');

for i = 1:length(bags)
    name = bags{i};    
    bag = eval(name);

        
    
    if ~isa(bag,'struct')
        continue
    end    
    filename = [name(1:strfind(name, '2014')-1) strrep(name(strfind(name,'2014'):end),'_','-')];
        
    if length(filename)==0
        filename = name;
    end
    
    fprintf('Processing [%s]\n', filename);
     
    t = bag.tf.time;
    yawQ     = bag.tf.world__cf_gt(:,4:7);
    yawGoalQ = bag.tf.world__goal(:,4:7);
    yawQ = [ yawQ(:,4) yawQ(:,1:3)];
    yawGoalQ = [ yawGoalQ(:,4) yawGoalQ(:,1:3)];
    [r,p,y] = quat2angle(yawQ,'XYZ');
    [rg,pg,yg] = quat2angle(yawGoalQ,'XYZ');
    
    rpy = unwrapRPY([r p y]);
    y = rpy(:,3)-2*pi;
    rpyg = unwrapRPY([rg pg yg]);
    yg = rpyg(:,3);
    
    
    
    yf = sgolayfilt( y, 3,51);

    figure(1); clf; hold on;
%     subplottight(2,1,1); cla; hold on;
    
    goalOffset =0.3;
    plot(bag.tf.time, rad2deg(y))
    plot(bag.tf.time+goalOffset, rad2deg(yg),'k--')
    plot(bag.tf.time, rad2deg(yf),'r') 
    grid on;
    title([strrep(filename(4:end),'_','-') ': 2d Traj vs Goal']);
    xlabel('Time [s]')
    ylabel('Rotation [^{\circ]} rad]')
    legend('Yaw_{quad}', 'Yaw_{goal}')
    xlim([0 30])
    
    %print(gcf, '-depsc', [name(4:end) '.eps']);
    print(gcf, '-dpng', [filename(4:end) '.png']);
    x=input('Press Enter to Continue')
    
end
