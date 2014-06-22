function [ rpy ] = unwrapRPY( rpy )
    %% Unwraps each column of rpy
    for x=1:size(rpy,2)
        xw = rpy(:,x);
        xu = xw;
        for i=2:length(xu)
            difference = xw(i)-xw(i-1);
            if difference > pi
                xu(i:end) = xu(i:end) - 2*pi;
            elseif difference < -pi
                xu(i:end) = xu(i:end) + 2*pi;
            end
        end        
        rpy(:,x) = xu;
    end
end

