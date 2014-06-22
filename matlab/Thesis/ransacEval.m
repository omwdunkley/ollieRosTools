eps = 0.2:0.1:0.8;
s = 1:8;
p=0.99;

data = zeros(length(eps), size(s,2));

for i=1:length(eps)
    data(i,:) = round(log(1-p)./log(1-(1-eps(i)).^s));
end

% eps * size
data

figure(1);
clf;
title('Ransac Iterations vs Model Size')
plot(eps,data')
legend('eps=10%','eps=20%','eps=30%','eps=40%','eps=50%','eps=60%','eps=70%')