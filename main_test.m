
%% Ben Cannon
function [t,z] = main_test
target_pose = [5; 5; pi()/2]; % INSERT TARGET POSE
tend=20;
tspan = [0 tend];
ic = [0; 0; pi/2];
opts = odeset('RelTol',1e-2,'AbsTol',1e-4);
[t,z] = ode45(@(t,z) diffdrive_test(t,z,target_pose, ic), tspan, ic, opts); % plot(z(:,1),z(:,2));grid
figure
plot(z(:,1),z(:,2));grid
xlabel('X Position','FontSize',14,'FontWeight','bold','Color','k'); 
ylabel('Y Position','FontSize',14,'FontWeight','bold','Color','k'); 
title('Robot Pose Versus Time','FontSize',16,'FontWeight','bold','Color','k');
end

