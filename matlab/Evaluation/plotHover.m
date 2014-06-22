function [] = plotHover()
load('/media/Evo/bags/ccrl/ccrl2/Hover/HOVER_2014-06-01-02-10-20_TFA.mat');
% load('/media/Evo/bags/ccrl/ccrl4/FullHover/fullHover_2014-06-02-17-25-48_TFA.mat');
% load('/media/Evo/bags/ccrl/ccrl4/FullHover/fullHoverCam_2014-06-02-18-05-45_TFA.mat');
% load('/media/Evo/bags/ccrl/ccrl4/FullHover/fullHoverCamOn_2014-06-02-18-58-32_TFA.mat');
load('Hover25FPS_passiveDelay75msNoise5cmCamon_2014-06-02-20-58-04_TFA.mat')
load('Hover25FPS_passiveDelay75msNoise5cm_2014-06-02-20-50-05_TFA.mat')
load('Hover25FPS_passiveDelay100msNoise10cm_2014-06-02-20-46-22_TFA.mat')
load('Hover25FPS_passiveDelay100ms_2014-06-02-20-34-49_TFA.mat')
load('Hover25FPS_passive_2014-06-02-20-33-17_TFA.mat')
load('Hover25FPS25_2014-06-02-20-31-33_TFA.mat')


A=TFAHOVER_2014_06_01_02_10_20;
% A=TFAfullHoverCamOn_2014_06_02_18_58_32;
% A=TFAfullHoverCam_2014_06_02_18_05_45;
% A=TFAfullHover_2014_06_02_17_25_48;
A=TFAHover25FPS25_2014_06_02_20_31_33;
A=TFAHover25FPS_passive_2014_06_02_20_33_17;
A=TFAHover25FPS_passiveDelay100ms_2014_06_02_20_34_49; %good
A=TFAHover25FPS_passiveDelay100msNoise10cm_2014_06_02_20_46_22;%explode
A=TFAHover25FPS_passiveDelay75msNoise5cm_2014_06_02_20_50_05;



A=TFAHover25FPS_passiveDelay75msNoise5cmCamon_2014_06_02_20_58_04;
% 30-70
range = 92:8697;


[tA, okA, endA] = getValidTime(A);
tMax = tA(endA);
okeA = find(okA); 
okeA = okeA(1:1:length(okeA));
xyzG = A.tf.world__goal(:,1:3);
xyz = A.tf.world__cf_gt(:,1:3);






%% BAT

colormap(jet)

x = smooth(xyz(:,1)-xyzG(:,1),11); %accidentally changed the goal a very little bit during experiment, fix
y = smooth(xyz(:,2)-xyzG(:,2),11);
z = smooth(xyz(:,3)-xyzG(:,3),11);
x = x-median(x(okA)).*0.85;
y = y-median(y(okA)).*0.85;
z = z-median(z(okA)).*0.85;

x = x(range);
y = y(range);
z = z(range);


dist = sqrt(gradient(x).^2+gradient(y).^2+gradient(z).^2);
vel = dist./0.01;
fprintf('MinVel: %f, MaxVel: %f MedianVel: %f\n', min(vel), max(vel), median(vel));


distXYZ = sqrt(x.^2 + y.^2 + z.^2);
rmseXYZ = sqrt(sum(distXYZ.^2)./length(range))
rmseXY
rmseZ
rmseYaw



%% 3d
v = [155,20; 0, 90; 0, 0];
for i = 1%:3
    figure(1);% clf; hold on;
    subplot(1,1,i); cla; hold on;
    colormap(jet); 

    xlim([-0.2,0.2]*2);
    ylim([-0.2,0.2]*2);
    zlim(1+[-0.2,0.2]*2);

    xl = xlim;
    yl = ylim;
    zl = zlim;

    o = ones(sum(okA),1);


    col = tA(okeA)./tMax; % time
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

return

%% 2d
figure(2); clf; hold on;
plot(tA,x,':k')
plot(tA,y,':k')
plot(tA,z,':k')
color_line(tA(okA),x(okA), col);
color_line(tA(okA),y(okA), col);
color_line(tA(okA),z(okA), col);


grid on;
% xlim([0, ceil(tMax)]);
% ylim([2.7, 3.8]);
xlabel('X [m]')
ylabel('Y [m]')
zlabel('Z [m]')



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

    startTime = 1.5;    
    endTime = startTime+1.0;
    
    startTime = 30 ./60; 
    endTime =  70./60;%time(end)
    
    
    ok = time<endTime & time>startTime;
    endInd=find(ok); endInd = endInd(end);   
    time = time - startTime;    

end


function [fr] = fitNice(time, data, d)
    [xData, yData] = prepareCurveData( time, data );
    ft = fittype( ['poly' num2str(d) ]);
    [fr, gof] = fit( xData, yData, ft );
end

