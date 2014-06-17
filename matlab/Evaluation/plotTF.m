
clear bag
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
    
%     smoothing
%     bag.tf.world__cf_gt(:,1:3) = [ ...
%         smooth(bag.tf.time,a(:,1),0.01,'loess') ...
%         smooth(bag.tf.time,a(:,2),0.01,'loess') ...
%         smooth(bag.tf.time,a(:,3),0.01,'loess')];
    
    %bag.tf.world__cf_gt(:,1:3) = sgolayfilt( bag.tf.world__cf_gt(:,1:3), 3,51);

    figure(1); clf; hold on;
%     subplottight(2,1,1); cla; hold on;
    subplot(2,1,1); cla; hold on;
    plot(bag.tf.time, bag.tf.world__cf_gt(:,1:3))
    plot(bag.tf.time, bag.tf.world__goal(:,1:3), '--')
    grid on;
    title([strrep(filename(4:end),'_','-') ': 2d Traj vs Goal']);
    xlabel('Time [s]')
    ylabel('Position [m]')
    legend('X_{quad}','Y_{quad}','Z_{quad}','X_{goal}','Y_{goal}','Z_{goal}')
    
    
        
    xyz = gradient(bag.tf.world__cf_gt(:,1:3))./0.01;
    figure(2); clf; hold on;
    plot(bag.tf.time, xyz,':');
    
    return
    
    
    


    %%    
%     subplottight(2,2,3); cla; hold on;
    subplot(2,2,3); cla; hold on;
    plot3(bag.tf.world__cf_gt(:,1), bag.tf.world__cf_gt(:,2),bag.tf.world__cf_gt(:,3),'b');
    plot3(bag.tf.world__goal(:,1), bag.tf.world__goal(:,2),bag.tf.world__goal(:,3),'k--');

    axis equal; grid on;
    title('3d Traj vs Goal');
    xlabel('X Position [m]')
    ylabel('Y Position [m]')
    zlabel('Z Position [m]')
    legend('Position_{quad}','Position_{goal}')
    axis([-2.5 2.5 -2.5 2.5 0 2])
    view(152,25)

%     subplottight(2,2,4); cla; hold on;
    subplot(2,2,4); cla; hold on;
    plot3(bag.tf.world__cf_gt(:,1), bag.tf.world__cf_gt(:,2),bag.tf.world__cf_gt(:,3),'b');
    plot3(bag.tf.world__goal(:,1), bag.tf.world__goal(:,2),bag.tf.world__goal(:,3),'r--');

    axis equal; grid on;
    title('3d Traj vs Goal');
    xlabel('X Position [m]')
    ylabel('Y Position [m]')
    zlabel('Z Position [m]')
    legend('Position_{quad}','Position_{goal}')
    axis([-2.5 2.5 -2.5 2.5 0 2]);
    view(180,90);
    filename = [name(1:strfind(name, '2014')-1) strrep(name(strfind(name,'2014'):end),'_','-')];
    

    
    
    
    
    
    print(gcf, '-dpng', [filename(4:end) '.png']);    
    %print(gcf, '-depsc', [name(4:end) '.eps']);
    x=input('Press Enter to Continue');
    
    
end


%% BARO
% figure(2)
% clf;
% asl=bag.baro.asl-mean(bag.baro.asl((bag.baro.time>10 & 12>bag.baro.time)))+0.5
% plot(bag.tf.time, bag.tf.world__cf_gt(:,3))
% grid on
% hold on;plot(bag.baro.time, asl,'r')
% plot(bag.tf.time, bag.tf.world__GOAL(:,3),'K--')
% plot(bag.tf.time, bag.tf.world__goal(:,3),'K--')
