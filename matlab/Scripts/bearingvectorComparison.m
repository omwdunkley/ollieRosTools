[x,y] = pol2cart(deg2rad(0:180), 1);
bv = [x; y; zeros(1,size(x,2))]';

figure(1); clf; hold on;
quiver(zeros(size(x,2),1), zeros(size(x,2),1), x', y');
axis equal;



e = []
for i = 1:size(bv,1)
    c=norm(cross(bv(1,:), bv(i,:)));
    d=1-dot(bv(1,:), bv(i,:)); 
    m=1-bv(1,:)* bv(i,:)';
    a= atan2(norm(cross(bv(1,:), bv(i,:))), dot(bv(1,:), bv(i,:)));
    n= norm(bv(1,:) - bv(i,:));
    s=sum((bv(1,:) - bv(i,:)).^2);
        
    e = [e; c d m a n s];
end

figure(2); clf; hold on; grid on; title('Reprojection Error'); xlabel('Angle [degrees] / Thresh Pixels'), ylabel('error / thresh');
plot(e(:,1),'r');
plot(e(:,2),'g');
plot(e(:,3),'b');
plot(e(:,4),'k');
plot(e(:,5),'c');
plot(e(:,6),'m');

px_width = 720;
fov      = deg2rad(110);
f_px = (px_width/2)/tan(fov/2)

plot(1.0 - cos(atan((0:180)./f_px)),':r.');
legend('||(AxB)||', '1-A.B', '1-A*B"','atan2(||(AxB)||, A.B)', '||(A-B)||', 'SUM((A-B).^2)', ['Thresh_' num2str(f_px)], 'Thresh480')