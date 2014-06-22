function [] = plotPIDThrust()
load('/media/Evo/bags/ccrl/ccrl2/XYZ/XYZ2cam_2014-06-01-01-49-27_TFA.mat');
A=TFAXYZ2cam_2014_06_01_01_49_27;



[tA, okA, endA] = getValidTime(A);
tMax = tA(endA);
okeA = find(okA); 
okeA = okeA(1:10:length(okeA));
cs = 0.1;


z  = A.tf.world__cf_gt(:,3);
zG = A.tf.world__goal(:,3);
zG(zG>0.5)=1.5;
zGi=find(abs(gradient(zG))>0);


%zg(zg>0.0)=pi;
%zgi=find(abs(gradient(yg))>0);

figure(1); clf; hold on;



xlim([0 tMax])
ylim([-0.5 2])
grid on;
xlabel('Time [s]')
ylabel('Z [m] / Throttle [%/100]')

plot(tA(okeA)-cs,    z(okeA),'-')
plot(tA(zGi)+0.17, zG(zGi),'--.k')

n = logical(A.pid.new);
nok = okA&n;
plot(tA(nok), A.pid.zpid_1(nok)./100,'r')
plot(tA(nok), A.pid.zpid_2(nok)./100,'g')
plot(tA(nok), A.pid.zpid_3(nok)./100,'c')
plot(tA(nok), A.pid.zpid_0(nok)./100,'m')

legend('Altitude', 'Set Point', 'P','I','D','Control')


matlab2tikz('pidZ.tikz', 'height', '\figureheight', 'width', '\figurewidth','extraAxisOptions','ticklabel shift={0.1cm}'); 



end


function [time, ok, endInd] = getValidTime(bag)
    time = bag.Time;
    
    startTime = 0
    endTime = time(end)
    startTime = 31;    
    endTime = 38;   
    
    
    ok = time<endTime & time>startTime;
    endInd=find(ok); endInd = endInd(end);   
    time = time - startTime;    

end
