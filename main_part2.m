
%% Ben Cannon
function [t,z] = main_part2
target_pose = [10; 10; pi()/2]; % INSERT TARGET POSE
tend=10;
tspan = [0 tend];
ic = [0; 0; 0];
opts = odeset('RelTol',1e-2,'AbsTol',1e-4);
[t,z] = ode45(@(t,z) diffdrive_part2(t,z,target_pose), tspan, ic, opts); % plot(z(:,1),z(:,2));grid
figure
plot(z(:,1),z(:,2));grid
xlabel('X Position','FontSize',14,'FontWeight','bold','Color','k'); 
ylabel('Y Position','FontSize',14,'FontWeight','bold','Color','k'); 
title('Robot Pose Versus Time','FontSize',16,'FontWeight','bold','Color','k');
end

