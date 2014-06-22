
clear bag
bags = who('TFA*');

for i = 1:length(bags)
    name = bags{i};    
    bag = eval(name);

        
    
    if ~isa(bag,'struct')
        continue
    end    
    filename = [name(1:strfind(name, '2014')-1) strrep(name(strfind(name,'2014'):end),'_','-')];
    if length(filename)==0
        filename = name;
    end
    
    fprintf('Processing [%s]\n', filename);
    figure(1); clf; hold on;
    subplot(1,1,1);cla;hold on;
    
    plot(bag.Time, bag.tf.world__cf_gt(:,3))
%     plot(bag.Time, bag.tf.world__goal(:,3),'--')
    n=logical(bag.baro.new);
    t=bag.Time(n);
    
    plot(t, bag.baro.aslRaw(n),':k')
    plot(t, bag.baro.asl(n),'r')
    plot(t, bag.baro.aslLong(n),'g')
    grid on;
    title([strrep(filename(4:end),'_','-') ': 2d Traj vs Goal']);
    
    xlabel('Time [s]')
    ylabel('Position [m]')
    legend('Z_{quad}','ASLRaw','ASL','ASLLong')

   
    
    print(gcf, '-dpng', [filename(4:end) '.png']);        
    
    x=input('Press Enter to Continue');
    
    
end