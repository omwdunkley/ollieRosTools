function [] = plotPIDX()
load('/media/Evo/bags/ccrl/ccrl3/yaw/YAW_2014-06-02-01-19-24_TFA.mat');
A=TFAYAW_2014_06_02_01_19_24;



[tA, okA, endA] = getValidTime(A);
tMax = tA(endA);
okeA = find(okA); 
okeA = okeA(1:10:length(okeA));



yawQ     = A.tf.world__cf_gt(:,4:7);
yawGoalQ = A.tf.world__goal(:,4:7);
yawQ = [ yawQ(:,4) yawQ(:,1:3)];
yawGoalQ = [ yawGoalQ(:,4) yawGoalQ(:,1:3)];
[r,p,y] = quat2angle(yawQ,'XYZ');
[rg,pg,yg] = quat2angle(yawGoalQ,'XYZ');

rpy = unwrapRPY([r p y]);
y = rpy(:,3);

rpyg = unwrapRPY([rg pg yg]);
yg = rpyg(:,3);

yg(yg>0.0)=pi;
ygi=find(abs(gradient(yg))>0);

figure(1); clf; hold on;


plot(tA(okeA), y(okeA))
plot(tA(ygi)+0.05, yg(ygi),'--.k')

xlim([0 tMax])
grid on;
xlabel('Time [s]')
ylabel('Yaw [rad]')


n = logical(A.pid.new);
nok = okA&n;


% plot(tA(:), medfilt1(A.pid.rzpid_0(:),1),'r')
plot(tA(nok), medfilt1( minmax(-deg2rad(A.pid.rzpid_1(nok)),2*pi),01),'r')
% plot(tA(:), medfilt1(A.pid.rzpid_2(:),1)./30,'m')
plot(tA(nok), medfilt1( minmax(deg2rad(A.pid.rzpid_3(nok)),2*pi),1),'c')
% plot(tA(n), medfilt1( minmax(-deg2rad(A.pid.rzpid_0(n)),2*pi),1),'k')

set(gca,'YTick',-2*pi:pi:2*pi)
set(gca,'YTickLabel',{'$-2\pi$','$-1\pi$','$0$','$1\pi$','$2\pi$'})

legend('Yaw', 'Set Point', 'P','D')


matlab2tikz('pidYaw.tikz', 'height', '\figureheight', 'width', '\figurewidth','extraAxisOptions','ticklabel shift={0.1cm}');



end

function [v] = minmax(v,t)
 v = min(t,max(-t,v));
end

function [time, ok, endInd] = getValidTime(bag)
    time = bag.Time;
    
    startTime = 0
    endTime = time(end)
     startTime = 9.8;    
     endTime = 11.5;   
    
    
    ok = time<endTime & time>startTime;
    endInd=find(ok); endInd = endInd(end);   
    time = time - startTime;    

end
