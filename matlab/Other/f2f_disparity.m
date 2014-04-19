function [ disparity ] = f2f_disparity(f1, f2)
%Frame to frame disparity
%   Computes the outlier-robust median disparity of features correspondances 
%   in the image plane between the current frame and a key frame


% Compute median disparity
fdiff = f2-f1;
disparity = median(hypot(fdiff(:,1), fdiff(:,2)));

end

