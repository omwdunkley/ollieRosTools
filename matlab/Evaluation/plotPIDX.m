function [] = plotPIDX()
load('/media/Evo/bags/ccrl/ccrl2/X/PIDXY_2014-05-31-22-00-18_TFA.mat');
% load('/media/Evo/bags/ccrl/ccrl2/X/X_2014-05-31-23-48-06_TFA.mat');
% load('/media/Evo/bags/ccrl/ccrl3/xyz/XYZ3_2014-06-02-00-46-10_TFA.mat')
A=TFAPIDXY_2014_05_31_22_00_18;
% A=TFAX_2014_05_31_23_48_06;
% A=TFAXYZ3_2014_06_02_00_46_10;


[tA, okA, endA] = getValidTime(A);
tMax = tA(endA);
okeA = find(okA); 
okeA = okeA(1:10:length(okeA));
xyzG = A.tf.world__goal(:,1:3);
xyz = A.tf.world__cf_gt(:,1:3)-0.1;

quat = A.tf.world__cf_gt(:,4:7);
[y,p,r] = quat2angle(quat);
r = rad2deg(r);

% roll = A.stabilizer.roll;


cs = 0.10; %control shift (time)
xs = 0.10;  % pos shift (x)
xG = xyzG(:,1);
xG(xG>-1)=1;

xGi=find(abs(gradient(xG))>0);

x  = xyz(:,1)+xs;

n = logical(A.pid.new);

figure(1); clf; hold on;
plot(tA(okeA)-cs,    x(okeA))
plot(tA(xGi), xG(xGi),'--.k')
xlim([0 tMax])
grid on;
xlabel('Time [s]')
ylabel('X [m]')

% plot(tA(:), r(:),'r')
% plot(tA(:), roll(:),'g')

% plot(tA(:), medfilt1(A.pid.xpid_0(:),1)./30,'g')
plot(tA(okA&n), medfilt1(deg2rad(A.pid.xpid_1(okA&n)),1),'r')
% plot(tA(:), medfilt1(A.pid.xpid_2(:),1)./30,'m')
plot(tA(okA&n), medfilt1(deg2rad(A.pid.xpid_3(okA&n)),1),'-c')





legend('Position', 'Set Point', 'P','D')



matlab2tikz('pidx.tikz', 'height', '\figureheight', 'width', '\figurewidth','extraAxisOptions','ticklabel shift={0.1cm}');



end


function [time, ok, endInd] = getValidTime(bag)
    time = bag.Time;
    
    startTime = 0
    endTime = time(end)
     startTime = 61;    
     endTime = 68;   
    
    
    ok = time<endTime & time>startTime;
    endInd=find(ok); endInd = endInd(end);   
    time = time - startTime;    

end
