function [ cmap ] = depth_map( dist,min_dist, max_dist)
% Returns a scaled colored map between min_dist and max_dist. If min and
% max are equal, compute the min and max from the input set.

if min_dist==max_dist
    min_dist = min(dist(:));
    max_dist = max(dist(:));
end
dist = max(min(dist,max_dist), min_dist);

dist = dist - min(dist);
dist = dist ./ (max_dist-min_dist);

cmap = flipud(colormap(jet(256)));
cmap = cmap(max(1,ceil(dist*256)),:);





end

