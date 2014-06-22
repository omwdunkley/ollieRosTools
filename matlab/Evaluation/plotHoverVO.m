function [] = plotHoverVO()

clear; close all;

r = 0.3;

load('/media/Evo/bags/ccrl/home6/VOHover_2014-06-19-02-37-32_TFA.mat')
A=TFAVOHover_2014_06_19_02_37_32;


[tA, okA, endA] = getValidTime(A);
tMax = tA(endA);
okeA = find(okA); 
okeA = okeA(1:1:length(okeA));

xyzG = A.tf.world__goal(:,1:3); % goal
xyz = A.tf.world__Q0(:,1:3);    % ground truth
xyzV = A.tf.world__cf_gt(:,1:3);%




%% BAT

colormap(jet)

% Align frames
x = smooth(xyz(:,1)-xyzG(:,1),11); %accidentally changed the goal a very little bit during experiment, fix
y = smooth(xyz(:,2)-xyzG(:,2),11);
z = smooth(xyz(:,3)-xyzG(:,3),11);
x = x-median(x(okeA));
y = y-median(y(okeA));
z = z-median(z(okeA));


xV = smooth(xyzV(:,1)-xyzG(:,1),11); %accidentally changed the goal a very little bit during experiment, fix
yV = smooth(xyzV(:,2)-xyzG(:,2),11);
zV = smooth(xyzV(:,3)-xyzG(:,3),11);
xV = xV-median(xV(okeA));
yV = yV-median(yV(okeA));
zV = zV-median(zV(okeA));


dist = sqrt(gradient(x).^2+gradient(y).^2+gradient(z).^2);
vel = dist./0.01;
fprintf('MinVel: %f, MaxVel: %f MedianVel: %f\n', min(vel), max(vel), median(vel));


distXYZ = sqrt(x.^2 + y.^2 + z.^2);
rmseXYZ = sqrt(sum(distXYZ.^2)./length(distXYZ))
% rmseXY
% rmseZ
% rmseYaw



%% 3d
v = [155,20; 0, 90; 0, 0];
for i = 1:3
    figure(1+i); clf; hold on;
    %subplot(1,1,i); cla; hold on;
    colormap(jet); 

    xlim([-r,r]);
    ylim([-r,r]);
    zlim(1+[-r,r]);

    xl = xlim;
    yl = ylim;
    zl = zlim;

    o = ones(sum(okA),1);

    col = vel(okeA);

    color_line3(x(okeA),y(okeA),1+z(okeA),col,'linewidth',2);
    % s(2)=color_line3(o.*xl(1),y(okA),z(okA), col);
    % s(3)=color_line3(x(okA),o.*yl(1),z(okA), col);
    % s(4)=color_line3(x(okA),y(okA),o*zl(1), col);
    grid on;

    axis square
    xlabel('X [m]')
    ylabel('Y [m]')
    zlabel('Z [m]')
    
    view(v(i,1),v(i,2));
    plotSpot(0,0,1)
    %matlab2tikz(['hover25Fps5cmCam' num2str(i) '.tikz'], 'height', '\figureheight', 'width', '\figurewidth','extraAxisOptions','ticklabel shift={0.1cm}');
end

%% 2d
figure(1); clf; hold on;

sp(1)=subplot(3,1,1); cla; hold on; grid on;
line([0 tMax]',[0 0],'color','k','linestyle','--')
plot(tA,x,'r')
plot(tA,xV, 'g')
xlabel('Time [minutes]');
ylabel('X [m]')
legend('Goal','Ground Truth','Visual Odometry');
ylim([-r r])

sp(2)=subplot(3,1,2); cla; hold on; grid on;
line([0 tMax]',[0 0],'color','k','linestyle','--')
plot(tA,y,'r')
plot(tA,yV,'g')
xlabel('Time [minutes]');
ylabel('Y [m]')
legend('Goal','Ground Truth','Visual Odometry');
ylim([-r r])

sp(3)=subplot(3,1,3); cla; hold on; grid on;
line([0 tMax]',[1 1],'color','k','linestyle','--')
plot(tA,1+z,'r')
plot(tA,1+zV,'g')
xlabel('Time [minutes]');
ylabel('Z [m]')
legend('Goal','Ground Truth','Visual Odometry');
ylim(1+[-r r])

linkaxes(sp,'x')


xlim([0 tMax])
xlim([0 1])



end


function [] = plotSpot(x,y,z)
    xl = xlim;
    yl = ylim;
    zl = zlim;
    plot3(x,y,z,'kd', 'markersize',2, 'linewidth', 2); % center
    plot3(xl(1),y,z,'kd', 'markersize',2, 'linewidth', 2); % projections
    plot3(x,yl(1),z,'kd', 'markersize',2, 'linewidth', 2);
    plot3(x,y,zl(1),'kd', 'markersize',2, 'linewidth', 2);
    line([xl(1) xl(2)],[y y],[z z],'linestyle','--','color',[0.35,0.35,0.35])
    line([x x],[yl(1) yl(2)],[z z],'linestyle','--','color',[0.35,0.35,0.35])
    line([x x],[y y],[zl(1) zl(2)],'linestyle','--','color',[0.35,0.35,0.35])    
end


function [time, ok, endInd] = getValidTime(bag)
    time = bag.Time./60;

    %startTime = 1.5;    
    %endTime = startTime+1.0;
    
    %startTime = 30 ./60; 
    %endTime =  70./60;%time(end)
    
    startTime =0;
    endTime = time(end);
    
    
    % just take the first minute
    startTime = 0.15;
    endTime = 1 + startTime;
    
    
    ok = time<endTime & time>startTime;
    endInd=find(ok); endInd = endInd(end);   
    time = time - startTime;    

end


function [fr] = fitNice(time, data, d)
    [xData, yData] = prepareCurveData( time, data );
    ft = fittype( ['poly' num2str(d) ]);
    [fr, gof] = fit( xData, yData, ft );
end

