bag = bag2013_12_12_15_32_59slowtrajpart;


figure(1); clf; hold on;
plot(bag.tf.time, [bag.tf.world__cf_gt(:,1:3)])
plot(bag.tf.time, [bag.tf.world__goal(:,1:3)],'--')
grid on;
title('2d Traj vs Goal');
xlabel('Time [s]')
ylabel('Position [m]')
legend('X_{quad}','Y_{quad}','Z_{quad}','X_{goal}','Y_{goal}','Z_{goal}')

%%
figure(2); clf; hold on;
plot3(bag.tf.world__cf_gt(:,1), bag.tf.world__cf_gt(:,2),bag.tf.world__cf_gt(:,3),'b');
plot3(bag.tf.world__goal(:,1), bag.tf.world__goal(:,2),bag.tf.world__goal(:,3),'k--');

axis equal; grid on;
title('3d Traj vs Goal');
xlabel('X Position [m]')
ylabel('Y Position [m]')
zlabel('Z Position [m]')
legend('Position_{quad}','Position_{goal}')
axis([-2.5 2.5 -2.5 2.5 0 2.5])