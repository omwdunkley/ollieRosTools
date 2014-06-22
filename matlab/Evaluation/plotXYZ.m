function [] = plotXYZ()
    clear;
    close all;
%      load('XYZ2cam_2014-06-01-01-47-42_TFA.mat')
     load('XYZ2cam_2014-06-01-01-49-27_TFA.mat') % best
%      load('XYZ2cam_2014-06-01-01-51-47_TFA.mat')
%      load('XYZ2CamVid_2014-06-01-02-47-05_TFA.mat')
%      load('XYZ2CamVid_2014-06-01-02-48-24_TFA.mat')
%      load('XYZ2CamVid_2014-06-01-02-51-02_TFA.mat')
%      load('XYZ3cam_2014-06-01-01-59-03_TFA.mat')
%      load('XYZ3cam_2014-06-01-02-00-38_TFA.mat')
%      load('XYZ3cam_2014-06-01-02-01-32_TFA.mat')
%      load('XYZ_good2014-06-01-00-46-24_TFA.mat')
%      load('XYZ_good2014-06-01-00-49-29_TFA.mat')

    v = [155,20; 0, 90; 0, 0];


    clear bag
    bags = who('TFA*');

    for i = 1:length(bags)
        name = bags{i}
        A = eval(name);

        if ~isa(A,'struct')
            continue
        end  


        
        

        for j = 1:3
            [tA, okA, endA] = getValidTime(A);
            tMax = tA(endA);
            okeA = find(okA); 
            okeA = okeA(1:4:end);
            
            
            [meanVel, maxVel, vel] = velStat(A.tf.world__cf_gt(okA,1:3));
            meanVel
            maxVel
            figure(21); clf; hold on; plot(tA(okA), vel);

            
            
            xyz  = A.tf.world__cf_gt(okeA,1:3);
            xyzG = A.tf.world__goal(okeA,1:3);

            vel = min(0.015*4,sqrt(gradient(xyz(:,1)).^2+gradient(xyz(:,2)).^2+gradient(xyz(:,3)).^2))./0.01;
            

            %plot3(xyzG(:,1),xyzG(:,2),xyzG(:,3),'k.--')





            l=-1.25;
            r=1.25;
            u=1.5;
            d=0.5;
            f = 1.25;
            b = -1.25;
            m = 0.0;

            g = [m f d; l f d; l b d; r b d; r f d; r f u; l f u; l b u; m b u; m m u; m m 0.0];
            
            %rmseXYZ = RMSE_traj(g,xyz)



            figure(j); clf; hold on;
    %         subplot(2,2,1); cla; hold on;


            plot3(g(:,1),g(:,2),g(:,3),'.--k')    
            %plot3(xyz(:,1),xyz(:,2),xyz(:,3),'b')
            color_line3(xyz(:,1), xyz(:,2), xyz(:,3), vel, 'linewidth',4);
            %legend('Goal Trajectory','Quad Trajectory')
            %view(152,25)
            
            xlim([-2 2])
            ylim([-2 2])
            zlim([0 2])
            grid on;
            xlabel('X [m]')
            ylabel('Y [m]')
            zlabel('Z [m]')     
            axis square
            view(v(j,1),v(j,2))
            view

    %         subplot(2,2,2); cla; hold on;
    %         plot3(g(:,1),g(:,2),g(:,3),'.--k')    
    %         %plot3(xyz(:,1),xyz(:,2),xyz(:,3),'b')
    %         color_line3(xyz(:,1), xyz(:,2), xyz(:,3), vel);
    %         legend('Quad Trajectory', 'Goal Trajectory')
    %         view(180,0)
    %         xlim([-2 2])
    %         ylim([-2 2])
    %         zlim([0 2])
    %         grid on;
    %         xlabel('X [m]')
    %         ylabel('Y [m]')
    %         zlabel('Z [m]')        
    %        
    %         subplot(2,2,3); cla; hold on;
    %         plot3(g(:,1),g(:,2),g(:,3),'.--k')    
    %         %plot3(xyz(:,1),xyz(:,2),xyz(:,3),'b')
    %         color_line3(xyz(:,1), xyz(:,2), xyz(:,3), vel);
    %         legend('Quad Trajectory', 'Goal Trajectory')
    %         view(-90,0)
    %         xlim([-2 2])
    %         ylim([-2 2])
    %         zlim([0 2])
    %         grid on;
    %         xlabel('X [m]')
    %         ylabel('Y [m]')
    %         zlabel('Z [m]')                




            filename = [name(1:strfind(name, '2014')-1) strrep(name(strfind(name,'2014'):end),'_','-')];
            %print(gcf, '-dpng', [filename(4:end) 'XYZ_' num2str(j) '_.png']);    
            %print(gcf, '-depsc', [name(4:end) '.eps']);
            %x=input('Press Enter to Continue');


            %matlab2tikz(['XYZ' num2str(j) '.tikz'], 'height', '\figureheight', 'width', '\figurewidth','extraAxisOptions','ticklabel shift={0.1cm}'); 
            

            
            
            return
        end
    end


end


function [time, ok, endInd] = getValidTime(bag)
    time = bag.Time;
    
    startTime = 18;
    endTime = 51% 51.5; 
    %endTime = time(end);
    %startTime = 0;    
      
    
    
    ok = time<endTime & time>startTime;
    endInd=find(ok); endInd = endInd(end);   
    time = time - startTime;    

end
