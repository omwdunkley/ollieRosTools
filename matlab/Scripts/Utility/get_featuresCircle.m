function [ points ] = get_features(percent, max_nr)
%GET_POINTS Returns a list of 3d points in global coordinates

    % PARAMS
    limits =  [-8 8 -1 -6 -5 5]; % xmin xmax ymin ymax zmin zmax
    radLim = [7 11];
    zLim = [-1 3];
    
    
    % Get xyz positions
    [x,y]=pol2cart(linspace(0,2*pi, 3*max_nr), 0.5*(radLim(1)+radLim(2))+randn(1,3*max_nr)*(radLim(2)-radLim(1))*0.25);
    z = 0.5*(zLim(1)+zLim(2))+randn(1,3*max_nr)*(zLim(2)-zLim(1))*0.5;
    
    
    NR = size(x(:),1);    
    
    nr = min(ceil(NR./100 * percent), max_nr);
        
    ids = randsample(NR,nr) ;
    points = [x(:) y(:) z(:) ones(NR,1)];
    %points = [x(:) y(:) z(:)];
    points = points(ids,:)';

    

end

