function [ points ] = get_features(percent, max_nr)
%GET_POINTS Returns a list of 3d points in global coordinates

    % PARAMS
    limits =  [-8 8 -1 -6 -5 5]; % xmin xmax ymin ymax zmin zmax
    xres = 0.01;
    yres = 0.01;
    zres = 0.01;
%     percent = 10;
%     max_nr = 200;
    

    % Wall
    
    
    % Get xyz positions
    [x, z] = meshgrid(limits(1):xres:limits(2), limits(5):zres:limits(6));%   x = x(:); y=y(:);        
    y = cos(x-z ).* cos(x./7).*sin(z/4);
    y = y - min(y(:));
    y = y./max(y(:));
    y = y.*abs(limits(3) - limits(4));
    y = y + limits(4);
    y = round(y./yres).*yres;
   
    
    NR = size(x(:),1);    
    
    nr = min(ceil(NR./100 * percent), max_nr);
        
    ids = randsample(NR,nr) ;
    points = [x(:) y(:) z(:) ones(NR,1)];
    %points = [x(:) y(:) z(:)];
    points = points(ids,:)';

    

end

