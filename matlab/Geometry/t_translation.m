function [ xyzh ] = t_translation( t )
    xyzh = squeeze(t(1:4,4,:));
end

