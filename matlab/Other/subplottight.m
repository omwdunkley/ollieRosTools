function subplottight(n,m,i)
    [c,r] = ind2sub([m n], i);
    w = (1/m);
    h = (1/n);
    l = (c-1)/m;
    b = 1-(r)/n;
    subplot('Position', [l+w*0.05 b+h*0.05 w*0.9 h*0.9])