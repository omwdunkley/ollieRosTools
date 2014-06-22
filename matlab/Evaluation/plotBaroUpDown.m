load('Baro2_2014-05-30-17-29-29_TFA.mat')
load('Baro2_2014-05-30-17-49-36_TFA.mat')
load('Baro2_2014-05-30-17-54-48_TFA.mat')
load('Baro2_2014-05-30-18-36-24_TFA.mat')
load('Baro3_2014-05-30-18-37-01_TFA.mat')
load('Baro4_2014-05-30-18-39-36_TFA.mat')
load('Baro4_2014-05-30-18-40-16_TFA.mat')
load('Baro5_2014-05-30-18-46-01_TFA.mat')
load('Baro5_2014-05-30-18-48-01_TFA.mat')


bag = TFABaro5_2014_05_30_18_46_01;



% Time in minutes
time=bag.Time;%./60.;
t0 = 0;%/60.;
t1 = 55;%/60.;


ok = 1:25:length(time);

b =  [bag.baro.aslRaw  bag.baro.asl  bag.baro.aslLong];
z = bag.tf.world__cf_gt(:,3);

figure(1); clf; hold on;
plot(time(ok), z(ok))


n=logical(bag.baro.new);

n = find(n); n = n(1:2:end);
t=time(n);

plot(t, b(n,1),'-','color',[0.6 0.6 0.6])
plot(t, b(n,2),'r')
plot(t, b(n,3),'g')


grid on;

xlabel('Time [s]')
ylabel('Altitude [m]')
legend('Qualisys','Baro_{Raw}','Baro_{Compensated}','Baro_{Orig}','Location','SouthEast')

xlim([t0 t1])
ylim([-2.5 2.5]-0.25)

figure1=gcf;

% % Create textarrow
% annotation(figure1,'textarrow',[0.512323943661972 0.532863849765258],...
%     [0.506712082262209 0.551413881748072],'TextEdgeColor','none',...
%     'String',{'Unreliable close to the ground'});
% 
% % Create textarrow
% annotation(figure1,'textarrow',[0.326291079812207 0.306338028169014],...
%     [0.8068669527897 0.566523605150215],'TextEdgeColor','none',...
%     'HorizontalAlignment','left',...
%     'String',{'Lift off'});
% 
% % Create textarrow
% annotation(figure1,'textarrow',[0.264084507042254 0.286384976525822],...
%     [0.736051502145923 0.566523605150215],'TextEdgeColor','none',...
%     'String',{'Motors start'});
% 
% % Create textarrow
% annotation(figure1,'textarrow',[0.762323943661972 0.779389671361504],...
%     [0.571961373390558 0.794450643776823],'TextEdgeColor','none',...
%     'String',{'Well Compensated'});
% 
% % Create textarrow
% annotation(figure1,'textarrow',[0.701338028169015 0.718403755868547],...
%     [0.483291845493562 0.705781115879828],'TextEdgeColor','none',...
%     'String',{'Drift'});


matlab2tikz('baroUpDown.tikz', 'height', '\figureheight', 'width', '\figurewidth','extraAxisOptions','ticklabel shift={0.1cm}');

print(gcf, '-dpng', 'baroUpDown.png');        
