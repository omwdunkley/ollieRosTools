function [ t ] = t_inv( T )


  R = T(1:3,1:3);  
  t = [R'     -R'*T(1:3,4)
       0 0 0  1          ];
end

