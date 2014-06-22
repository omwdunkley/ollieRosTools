function [ meanV, maxV, vel ] = velStat(xyz)
vel = sqrt(gradient(xyz(:,1)).^2+gradient(xyz(:,2)).^2+gradient(xyz(:,3)).^2)./0.01;
maxV = max(vel);
meanV = mean(vel);


end

