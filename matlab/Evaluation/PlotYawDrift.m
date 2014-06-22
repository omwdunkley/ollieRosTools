% Yaw drift

load('/media/Evo/bags/ccrl/Yaw_2014-06-11-15-06-19_EQ.mat')
bag = EQ_Yaw_2014_06_11_15_06_19;

f = find(logical(bag.stabilizer.new));
%f = f(1:50:end);


time = bag.Time(f)
dtime = gradient(time);

rpy = [bag.stabilizer.roll(f) bag.stabilizer.pitch(f) bag.stabilizer.yaw(f)];
drpy = gradient(rpy);



figure(1); clf; hold on;
plot(time, drpy);

return
plot(bag.Time(f)-11,(bag.stabilizer.roll(f))./bag.Time(f), 'r');
plot(bag.Time(f)-11,(bag.stabilizer.pitch(f))./bag.Time(f), 'g');
plot(bag.Time(f)-11,(bag.stabilizer.yaw(f)-0.2)./bag.Time(f), 'b');
legend('Roll','Pitch','Yaw')

ylim([0,0.12]); xlim([0,275]); 
grid on; 
xlabel('Time [s]'); 
ylabel('Drift [Deg/s]')

matlab2tikz(['yawDrift' num2str(i) '.tikz'], 'height', '\figureheight', 'width', '\figurewidth','extraAxisOptions','ticklabel shift={0.1cm}');