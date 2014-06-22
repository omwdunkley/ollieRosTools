function [] = plotHoverDischarge()
load('/media/Evo/bags/ccrl/ccrl4/FullHover/fullHover_2014-06-02-17-25-48_TFA.mat');
load('/media/Evo/bags/ccrl/ccrl4/FullHover/fullHoverCam_2014-06-02-18-05-45_TFA.mat');
load('/media/Evo/bags/ccrl/ccrl4/FullHover/fullHoverCamOn_2014-06-02-18-58-32_TFA.mat');
A=TFAfullHover_2014_06_02_17_25_48;
B=TFAfullHoverCam_2014_06_02_18_05_45;
C=TFAfullHoverCamOn_2014_06_02_18_58_32;

[tA, okA, endA] = getValidTime(A);
[tB, okB, endB] = getValidTime(B);
[tC, okC, endC] = getValidTime(C);
tMax = max([tA(endA) tB(endB) tC(endC)]);

okeA=find(okA); okeA=okeA(1:20:length(okeA));
okeB=find(okB); okeB=okeB(1:20:length(okeB));
okeC=find(okC); okeC=okeC(1:20:length(okeC));



%% BAT
figure(1); clf; hold on;
plot(tA(okeA), smooth(A.pm.vbat(okeA),11),'color',[1.0,0.6,0.6])
plot(tB(okeB), smooth(B.pm.vbat(okeB),11),'color',[0.6,1.0,0.6])
plot(tC(okeC), smooth(C.pm.vbat(okeC),11),'color',[0.6,0.6,1.0])


fA=fitNice(tA(okA), A.pm.vbat(okA), 9);
fB=fitNice(tB(okB), B.pm.vbat(okB), 7);
fC=fitNice(tC(okC), C.pm.vbat(okC), 7);

a=plot( tA(okeA), feval(fA,tA(okeA)),'r');
b=plot( tB(okeB), feval(fB,tB(okeB)),'g');
c=plot( tC(okeC), feval(fC,tC(okeC)),'b');

plot(tA(endA), feval(fA, tA(endA)), 'rd', 'markersize',4, 'linewidth', 4)
plot(tB(endB), feval(fB, tB(endB)), 'gd', 'markersize',4, 'linewidth', 4)
plot(tC(endC), feval(fC, tC(endC)), 'bd', 'markersize',4, 'linewidth', 4)



grid on;
xlim([0, ceil(tMax)]);
ylim([2.7, 3.8]);
xlabel('Time [minutes]')
ylabel('Voltage [V]')
legend([a b c], {'No Camera', 'Camera Off','Camera On'});   

% matlab2tikz('voltageHover.tikz', 'height', '\figureheight', 'width', '\figurewidth');



%% THRUST
figure(2); clf; hold on;


plot(tA(okeA), smooth(double(A.stabilizer.thrust(okeA))./600,11),'color',[1.0,0.6,0.6])
plot(tB(okeB), smooth(double(B.stabilizer.thrust(okeB))./600,11),'color',[0.6,1.0,0.6])
plot(tC(okeC), smooth(double(C.stabilizer.thrust(okeC))./600,11),'color',[0.6,0.6,1.0])

fA = fitNice(tA(okA), double(A.stabilizer.thrust(okA))./600, 9);
fB = fitNice(tB(okB), double(B.stabilizer.thrust(okB))./600, 7);
fC = fitNice(tC(okC), double(C.stabilizer.thrust(okC))./600, 7);

a=plot(tA(okeA), feval(fA,tA(okeA)),'r');
b=plot(tB(okeB), feval(fB,tB(okeB)),'g');
c=plot(tC(okeC), feval(fC,tC(okeC)),'b');

plot(tA(endA), feval(fA, tA(endA)), 'rd', 'markersize',4, 'linewidth', 4)
plot(tB(endB), feval(fB, tB(endB)), 'gd', 'markersize',4, 'linewidth', 4)
plot(tC(endC), feval(fC, tC(endC)), 'bd', 'markersize',4, 'linewidth', 4)


grid on;
xlim([0, ceil(tMax)]);
ylim([60, 100]);
xlabel('Time [minutes]')
ylabel('Throttle [%]')
legend([a b c], {'No Camera', 'Camera Off','Camera On'},'Location','SouthEast');   

%matlab2tikz('thrustHover.tikz', 'height', '\figureheight', 'width', '\figurewidth');

%% motor

% front left back right
mA = [A.motor.m1, A.motor.m4, A.motor.m3, A.motor.m2]; mA = mA(okA&A.motor.new & abs(tA-2)<0.3,:);
mB = [B.motor.m1, B.motor.m4, B.motor.m3, B.motor.m2]; mB = mB(okB&B.motor.new & abs(tB-2)<0.3,:);
mC = [C.motor.m1, C.motor.m4, C.motor.m3, C.motor.m2]; mC = mC(okC&C.motor.new & abs(tC-2)<0.3,:);

[m,i]=min(abs(tA-2));
mA = mean(mA) ./ sum(mean(mA))%.* feval(fA,tA(i))*4
[m,i]=min(abs(tB-2));
mB = mean(mB) ./ sum(mean(mB))%.* feval(fB,tB(i))*4
[m,i]=min(abs(tC-2));
mC = mean(mC) ./ sum(mean(mC))%.* feval(fC,tC(i))*4


round(mA*200)/200;
round(mB*200)/200;
round(mC*200)/200;


at = find(A.pm.new);
bt = find(B.pm.new);
ct = find(C.pm.new);
datestr(tA(at(end))/24/60, 'HH:MM:SS.FFF')
datestr(tB(bt(end))/24/60, 'HH:MM:SS.FFF')
datestr(tC(ct(end))/24/60, 'HH:MM:SS.FFF')



end


function [time, ok, endInd] = getValidTime(bag)
    time = bag.Time./60;
    xyz = bag.tf.world__cf_gt(:,1:3);
    ok = abs(xyz(:,3)-0.75)<0.1;
    ok = imerode(ok, ones(100,1));   
      
    endInd = find(bag.pm.new);
    endInd = endInd(end);
    endTime = time(endInd);
    
    ok = ok & time<endTime;
        
    start = find(ok); 
    start = time(start(1));
    time = time - start;
    

end

function [fr] = fitNice(time, data, d)
    [xData, yData] = prepareCurveData( time, data );
    ft = fittype( ['poly' num2str(d) ]);
    [fr, gof] = fit( xData, yData, ft );
end

