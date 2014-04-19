function [ t ] = t_cons(T1, T2 )

 [a, b, c] = size(T1);
 [e, f, g] = size(T2);
 
 t = zeros(4,4,max(c,g));
 
 if c==g
     for i=1:c
         t(:,:,i) = T1(:,:,i) * T2(:,:,i);
     end
 end
 
 if c==1
     for i=1:g
         t(:,:,i) = T1(:,:,1) * T2(:,:,i);
     end
 end
 
 if g==1
     for i=1:c
         t(:,:,i) = T1(:,:,i) * T2(:,:,1);
     end
 end


end

