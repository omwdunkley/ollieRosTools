function [ t ] = tq2t( tq )
    
[r p y] = quat2angle(tq(:,4:7), 'xyz');
t = t_build(tq(:,1:3), [r p y]);

end

