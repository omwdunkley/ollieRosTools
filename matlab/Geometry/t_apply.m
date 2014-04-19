function t = t_apply(T, P)
    % Transform
    t = T*P;            
    % Normalise
    t = bsxfun(@rdivide, t,t(4,:));
end