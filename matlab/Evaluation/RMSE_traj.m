function [ rmse ] = RMSE_traj( waypoints, points )
    d = 0;

    ps = [];
    for li = 2:size(waypoints,1)
        % line segment
        a = waypoints(li-1,:);
        b = waypoints(li,:);
        ps = [ps; linspaceNDim(a,b,10000)'];
    end
    
    ps = unique(ps, 'rows');
    
    for pi = 1:size(points,1)
        p = points(pi,:);   
        d = d + min(normMat(bsxfun(@minus, p,ps)))^2;
    end
    
    d = sqrt(d/size(points,1));   
    
    rmse = d;

end

function n = normMat(M)
    n=sqrt(sum(M.^2,2));
end

