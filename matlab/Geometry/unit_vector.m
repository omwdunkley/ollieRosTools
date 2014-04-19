function [ uv ] = unit_vector( V )
% Makes 3xM or 4xM vecotr length 1

n = vnorm(V);
uv = bsxfun(@rdivide, V, n);

end

