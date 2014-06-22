load('/media/Evo/bags/ccrl/ccrl2/XYZ/XYZ2cam_2014-06-01-01-49-27_TFA.mat');
A=TFAXYZ2cam_2014_06_01_01_49_27;


t = A.Time;
z = A.tf.world__cf_gt(:,3);

figure(1); clf; hold on;
% plot(t,z)

dz = gradient(z)./0.01;

plot(t,[z, dz])