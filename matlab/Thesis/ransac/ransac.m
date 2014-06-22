function [ theta,rho] = ransac( pts,iterNum,thDist,thInlrRatio )
%RANSAC Use RANdom SAmple Consensus to fit a line
%	RESCOEF = RANSAC(PTS,ITERNUM,THDIST,THINLRRATIO) PTS is 2*n matrix including 
%	n points, ITERNUM is the number of iteration, THDIST is the inlier 
%	distance threshold and ROUND(THINLRRATIO*SIZE(PTS,2)) is the inlier number threshold. The final 
%	fitted line is RHO = sin(THETA)*x+cos(THETA)*y.
%	Yan Ke @ THUEE, xjed09@gmail.com

sampleNum = 2;
ptNum = size(pts,2);
thInlr = round(thInlrRatio*ptNum);
inlrNum = zeros(1,iterNum);
theta1 = zeros(1,iterNum);
rho1 = zeros(1,iterNum);
inlierBest = [];

for p = 1:iterNum
	% 1. fit using 2 random points
	sampleIdx = randIndex(ptNum,sampleNum);
	ptSample = pts(:,sampleIdx);
	d = ptSample(:,2)-ptSample(:,1);
	d = d/norm(d); % direction vector of the line
	
	% 2. count the inliers, if more than thInlr, refit; else iterate
	n = [-d(2),d(1)]; % unit normal vector of the line
	dist1 = n*(pts-repmat(ptSample(:,1),1,ptNum));
	inlier1 = find(abs(dist1) < thDist);
	inlrNum(p) = length(inlier1);
    
    
    figure(2); clf; hold on; grid on; axis equal; xlim([-150 150]);ylim([-150 150]);
    
    % Draw inliers
    plot(pts(1,inlier1), pts(2,inlier1),'gd','markersize',7,'linewidth',2)
    
     % Draw Selected Model
    plot(pts(1,:), pts(2,:),'k.','markersize',10)
    plot(ptSample(1,:), ptSample(2,:),'bo','markersize',7,'linewidth',2)  
    pdiff = ptSample(:,1)-ptSample(:,2);  
    ptE = [ptSample(:,1)+ 50*pdiff ptSample(:,1)- 50*pdiff];
    line(ptE(1,:), ptE(2,:),'color','b','linestyle','--')%,'b.','markersize',16)
    
    line(ptE(1,:)+n(1)*thDist, ptE(2,:)+n(2)*thDist,'color','g','linestyle','--','linewidth',2)%,'b.','markersize',16)
    line(ptE(1,:)-n(1)*thDist, ptE(2,:)-n(2)*thDist,'color','g','linestyle','--','linewidth',2)%,'b.','markersize',16)
%     line(ptE(1,:)+thDist, ptE(2,:)-thDist,'color','g','linestyle','--')%,'b.','markersize',16)
%     line(ptE(1,:)-thDist, ptE(2,:)-thDist,'color','g','linestyle','--')%,'b.','markersize',16)
    
    

    
    %legend('inlier','point','model','model','tolerance')
    axis([-1 1 -1 1].*100);
    if p == 1        
        x=input('continue?')
    end
    
    print(gcf, '-depsc', ['ransac_eps_' num2str(p) '_ ' num2str(inlrNum(p)) '.eps']);
    
    
	if length(inlier1) < thInlr, continue; end
	ev = princomp(pts(:,inlier1)');
	d1 = ev(:,1);
	theta1(p) = -atan2(d1(2),d1(1)); % save the coefs
	rho1(p) = [-d1(2),d1(1)]*mean(pts(:,inlier1),2);  
    

end

% 3. choose the coef with the most inliers
[val,idx] = max(inlrNum)
theta = theta1(idx);
rho = rho1(idx);

    k1 = -tan(theta);
    b1 = rho/cos(theta);
    ptSample = [-200 200; k1*-200+b1,k1*200+b1];


    figure(2); clf; hold on; grid on; axis equal; xlim([-150 150]);ylim([-150 150]);
    d = ptSample(:,2)-ptSample(:,1);
    d = d/norm(d);
    n = [-d(2),d(1)];
    dist1 = n*(pts-repmat(ptSample(:,1),1,ptNum));
	inlier1 = find(abs(dist1) < thDist);


    plot(pts(1,inlier1), pts(2,inlier1),'gd','markersize',5,'linewidth',2)

    % Draw Selected Model
    plot(pts(1,:), pts(2,:),'k.','markersize',10)
    plot(ptSample(1,:), ptSample(2,:),'bo','markersize',5,'linewidth',2)  
    pdiff = ptSample(:,1)-ptSample(:,2);  
    ptE = [ptSample(:,1)+ 50*pdiff ptSample(:,1)- 50*pdiff];
    line(ptE(1,:), ptE(2,:),'color','b','linestyle','-','linewidth',2)%,'b.','markersize',16)

    line(ptE(1,:)+n(1)*thDist, ptE(2,:)+n(2)*thDist,'color','g','linestyle','--','linewidth',2)%,'b.','markersize',16)
    line(ptE(1,:)-n(1)*thDist, ptE(2,:)-n(2)*thDist,'color','g','linestyle','--','linewidth',2)%,'b.','markersize',16)
    
    % least square fitting
    coef2 = polyfit(pts(1,inlier1),pts(2,inlier1),1);
    k2 = coef2(1);
    b2 = coef2(2);
    plot([-1000,1000],k2*[-1000, 1000]+b2, 'm','linewidth',1)
    
    

end