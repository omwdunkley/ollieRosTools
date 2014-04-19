%% Plots data saved before and after G2O
path = '~/Code/mine/ollieRosTools/FvsKF_Init_';



data=csvread([path 'bvp.csv']);
bvf  = data(:,1:3);
bvkf = data(:, 4:6);
p_w = data(:,7:9);
p_wG = data(:,10:12);
nr = min(size(p_w,1), 500);
h  = ones(nr,1);

p_w = [p_w(1:nr,:) h];
p_wG = [p_wG(1:nr,:) h];

bv_f = [bvf(1:nr,:) h];
bv_kf = [bvkf(1:nr,:) h];
bv_fG = [bvf(1:nr,:) h];
bv_kfG = [bvkf(1:nr,:) h];

data=csvread([path 'tf.csv']);
t_f = data(1:4, :);
t_kf = data(5:8, :);
t_fG = data(9:12, :);
t_kfG = data(13:16, :);
t_fi = t_inv(t_f);
t_kfi = t_inv(t_kf);
t_fiG = t_inv(t_fG);
t_kfiG = t_inv(t_kfG);

%% Points Local transforms
p_f = t_apply(t_fi, p_w')';
p_kf = t_apply(t_kfi, p_w')';
p_fG = t_apply(t_fiG, p_wG')';
p_kfG = t_apply(t_kfiG, p_wG')';

%% Depths

d_f = sqrt(sum(p_f(:,1:3).^2,2));
d_kf = sqrt(sum(p_kf(:,1:3).^2,2));
d_fG = sqrt(sum(p_fG(:,1:3).^2,2));
d_kfG = sqrt(sum(p_kfG(:,1:3).^2,2));

% apply to BV
bv_f(:,1:3) = bsxfun(@times, bv_f(:,1:3), d_f);
bv_kf(:,1:3) = bsxfun(@times, bv_kf(:,1:3), d_kf);
bv_fG(:,1:3) = bsxfun(@times, bv_fG(:,1:3), d_fG);
bv_kfG(:,1:3) = bsxfun(@times, bv_kfG(:,1:3), d_kfG);

% experiement with projection
%bv_f(:,1:3) = bsxfun(@rdivide, bv_f(:,1:3), bv_f(:,3));
%bv_kf(:,1:3) = bsxfun(@rdivide, bv_kf(:,1:3), bv_kf(:,3));



%% BV in global tfs

bv_f_w  = t_apply(t_f, bv_f')';
bv_kf_w = t_apply(t_kf, bv_kf')';
bv_f_wG  = t_apply(t_fG, bv_fG')';
bv_kf_wG = t_apply(t_kfG, bv_kfG')';


figure(1);clf;hold on;grid on;axis equal;xlabel('X');ylabel('Y');zlabel('Z');
%% Plot Points
plot3(p_w(1:nr,1),p_w(1:nr,2),p_w(1:nr,3),'or')
%% Plot Transforms
plot_frame(t_f, 0.5, 'F',3)
plot_frame(t_kf, 0.5, 'KF',3)
%% Plot Bearing vectors
plot3(bv_f_w(:,1), bv_f_w(:,2), bv_f_w(:,3),'.g')
plot3(bv_kf_w(:,1), bv_kf_w(:,2), bv_kf_w(:,3),'.b')
line([bv_f_w(:,1)  ones(nr,1)*t_f(1,4)]',  [bv_f_w(:,2)  ones(nr,1)*t_f(2,4)]', [bv_f_w(:,3)  ones(nr,1)*t_f(3,4)]', 'color','g','linestyle',':')
line([bv_kf_w(:,1) ones(nr,1)*t_kf(1,4)]', [bv_kf_w(:,2) ones(nr,1)*t_kf(2,4)]',[bv_kf_w(:,3) ones(nr,1)*t_kf(3,4)]','color','b','linestyle',':')

figure(2);clf;hold on;grid on;axis equal;xlabel('X');ylabel('Y');zlabel('Z');
%% Plot Points
plot3(p_wG(1:nr,1),p_wG(1:nr,2),p_wG(1:nr,3),'or')
%% Plot Transforms
plot_frame(t_fG, 0.5, 'F',3)
plot_frame(t_kfG, 0.5, 'KF',3)
%% Plot Bearing vectors
plot3(bv_f_wG(:,1), bv_f_wG(:,2), bv_f_wG(:,3),'.g')
plot3(bv_kf_wG(:,1), bv_kf_wG(:,2), bv_kf_wG(:,3),'.b')
line([bv_f_wG(:,1)  ones(nr,1)*t_fG(1,4)]',  [bv_f_wG(:,2)  ones(nr,1)*t_fG(2,4)]', [bv_f_wG(:,3)  ones(nr,1)*t_fG(3,4)]', 'color','g','linestyle',':')
line([bv_kf_wG(:,1) ones(nr,1)*t_kfG(1,4)]', [bv_kf_wG(:,2) ones(nr,1)*t_kfG(2,4)]',[bv_kf_wG(:,3) ones(nr,1)*t_kfG(3,4)]','color','b','linestyle',':')







%% reprojection error
sqrt(backproject_error(t_f , p_w', bv_f' ))
sqrt(backproject_error(t_kf , p_w', bv_kf' ))

sqrt(backproject_error(t_fG , p_wG', bv_fG' ))
sqrt(backproject_error(t_kfG , p_wG', bv_kfG' ))
