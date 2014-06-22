function testRansac

% real model coef
k = 1.5;
b = 01;
ptNum = 250;
outlrRatio = .75;
inlrStd = 1.5;
pts = genRansacTestPoints(ptNum,outlrRatio,inlrStd,[k b]);

X = -ptNum/2:ptNum/2;

err0 = sqrError(k,b,pts(:,1:ptNum*(1-outlrRatio)))

% RANSAC
iterNum = 300;
thDist = 8;
thInlrRatio = outlrRatio-1;
[t,r] = ransac(pts,iterNum,thDist,thInlrRatio);

k1 = -tan(t);
b1 = r/cos(t);
plot(X,k1*X+b1,'b','linewidth',2)
err1 = sqrError(k1,b1,pts(:,1:ptNum*(1-outlrRatio)))

% % least square fitting
% coef2 = polyfit(pts(1,:),pts(2,:),1);
% k2 = coef2(1);
% b2 = coef2(2);
% plot(X,k2*X+b2,'c','linewidth',1)
% err2 = sqrError(k2,b2,pts(:,1:ptNum*(1-outlrRatio)))


%plot(X,k*X+b,'k','linewidth',2)

axis([-1 1 -1 1].*100);
%legend('inlier','point','model','model','tolerance','tolerance')
print(gcf, '-depsc', 'ransac_final.eps');


end




function err = sqrError(k,b,pts)
%	Calculate the square error of the fit

theta = atan(-k);
n = [cos(theta),-sin(theta)];
pt1 = [0;b];
err = sqrt(sum((n*(pts-repmat(pt1,1,size(pts,2)))).^2));

end
