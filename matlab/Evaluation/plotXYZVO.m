function [] = plotXYZVO()
    clear;
    %close all;
    load('/media/BC1/THESIS_BAGS/ccrl7/VOSquare_2014-06-19-05-34-45_GOOD_TFA.mat')

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
            %okeA = find(okA); 
            %okeA = okeA(1:1:end);
            
            
%             [meanVel, maxVel, vel] = velStat(A.tf.world__Q0(okA,1:3));
%             meanVel
%             maxVel
%             figure(21); clf; hold on; plot(tA(okA), vel);

            
            
            xyzV  = bsxfun(@minus, A.tf.world__cf_gt(:,1:3), [1.8 2 0]);
            xyz   = bsxfun(@minus, A.tf.world__Q0(:,1:3), [1.8 2 0]);
            xyzG  = bsxfun(@minus, A.tf.world__goal(:,1:3), [1.8 2 0]);
            
            xG = xyzG(:,1);
            yG = xyzG(:,2);
            zG = xyzG(:,3);
            
            % Align frames
            smoothFactor = 11;
            x = smooth(xyz(:,1),smoothFactor); %accidentally changed the goal a very little bit during experiment, fix
            y = smooth(xyz(:,2),smoothFactor);
            z = smooth(xyz(:,3),smoothFactor);


            xV = smooth(xyzV(:,1),smoothFactor); %accidentally changed the goal a very little bit during experiment, fix
            yV = smooth(xyzV(:,2),smoothFactor);
            zV = smooth(xyzV(:,3),smoothFactor);

            vel = smooth(sqrt(gradient(xyz(:,1)).^2+gradient(xyz(:,2)).^2+gradient(xyz(:,3)).^2)./0.01, 51);
            vel = vel(okA);

            %plot3(xyzG(:,1),xyzG(:,2),xyzG(:,3),'k.--')





            l=-0.5;
            r=0.5;
            u=1.5;
            d=1.0;
            m = 0.0;

            g = [m,0,d; l,0,d; l 0 u; r 0 u; r 0 d; l 0 d];
            
            %rmseXYZ = RMSE_traj(g,xyz)



            figure(j); clf; hold on;
    %         subplot(2,2,1); cla; hold on;

            plot3(g(:,1),g(:,2),g(:,3),'.--k')    
            %plot3(xyzG(okA,1),xyzG(okA,2),xyzG(okA,3),'.--k')
            
            
            
            %plot3(xyz(:,1),xyz(:,2),xyz(:,3),'b')
            color_line3(x(okA),y(okA),z(okA), vel, 'linewidth',4);
            plot3(xV(okA),yV(okA),zV(okA), 'g', 'linewidth',2);
            %legend('Goal Trajectory','Quad Trajectory')
            %view(152,25)
            
%             xlim([-2 2])
%             ylim([-2 2])
%             zlim([0 2])
            xlim([-1.2 1.2])
            ylim([-1.2 1.2])
            zlim([0.5 2.0])            
            grid on;
            xlabel('X [m]')
            ylabel('Y [m]')
            zlabel('Z [m]')     
            axis square
            view(v(j,1),v(j,2))
            view
            
            legend('Goal','Ground Truth','Visual Odometry', 'location', 'east')

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
            

            figure(2); clf; hold on;
            
             % Get x Goal simplified
            lxG=max(1,find(logical(diff(abs(diff(xG(:)))>0.03)))+1);
            xG = xG(lxG);            
            xG(2) = 0;1.8;
            xG(3) = -0.5;1.3;
            xG(15) = -0.5; 1.3;
            txG = [tA(lxG(1:2:end)) tA(lxG(1:2:end))]';
            txG = txG(2:end);            
            xG = [xG; xG(end)];
            txG = [txG tMax];     
            
            
            lzG=max(1,find(logical(diff(abs(diff(zG(:)))>0.03)))+1);
            lzG = lzG(1:end-1);
            zG = zG(lzG);   
            zG(1:4) = 1.35;
            zG(5:end)= round(zG(5:end).*2)./2;
            %subplot(sp(3));            
            tzG = [tA(lzG(1:2:end)) tA(lzG(1:2:end))]';
            tzG = tzG(2:end)';            
            tzG(2:3) = [];
            zG(2:3) = [];
            zG(17:end)= [];
            tzG(17:end)= [];
            tzG(end) = tMax;  
            %plot(tzG,zG,'mp');
            %[tzG zG]
            
            
            
            
            
          

            sp(1)=subplot(3,1,1); cla; hold on; grid on;
            plot(txG,xG,'.--k')
            plot(tA,x,'r')
            plot(tA,xV, 'g')
            color_line3(tA,x,x, vel, 'linewidth',2);
            xlabel('Time [s]');
            ylabel('X [m]')
            legend('Goal','Ground Truth','Visual Odometry');
            %ylim([-r r])

            sp(2)=subplot(3,1,2); cla; hold on; grid on;
            %plot(txG,ones(length(txG),1)*2,'.--k')
            line([0 tMax]', [0 0],'color','k','linestyle','--');
            plot(tA,y,'r')
            plot(tA,yV,'g')
            xlabel('Time [s]');
            ylabel('Y [m]')
            legend('Goal','Ground Truth','Visual Odometry');
            %ylim([-r r])

            sp(3)=subplot(3,1,3); cla; hold on; grid on;
            plot(tzG,zG,'.--k')
            plot(tA,z,'r')
            plot(tA,zV,'g')
            xlabel('Time [s]');
            ylabel('Z [m]')
            legend('Goal','Ground Truth','Visual Odometry');
            %ylim(1+[-r r])

            linkaxes(sp,'x')


            xlim([0 tMax])
            %xlim([0 1])
            

            
            return
        end
    end


end


function [time, ok, endInd] = getValidTime(bag)
    time = bag.Time;
    
    startTime = 55+50-20+5.75;
    endTime = startTime+20-1.25;%time(end);
    
    
    ok = time<endTime & time>startTime;
    endInd=find(ok); endInd = endInd(end);   
    time = time - startTime;    

end
