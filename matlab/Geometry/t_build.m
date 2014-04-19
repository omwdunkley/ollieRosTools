function [ t ] = build_t(xyz,rpy )

    t = zeros(4,4,size(xyz,1));
    for i = 1:size(xyz,1)
        t(:,:,i) = [[angle2dcm(rpy(i,1),rpy(i,2),rpy(i,3),'XYZ'); 0 0 0] [xyz(i,1); xyz(i,2); xyz(i,3); 1]];
    end
end

