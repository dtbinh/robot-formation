function dzdt=diffdrive(t,y,rt,RD,lt,LD)
% This defines the right hand side of the differential equation model for
% the simple car with two wheels. RD,rd is the control for the right wheel
% and LD,ld is the control for the left wheel
r=0.1; % Radius of wheel
L=0.5; % Distance between wheels
RD = interp1(rt,RD,t); % Interpolate the data set at time t
LD = interp1(lt,LD,t); % Interpolate the data set at time t
dzdt = [
(r*(RD+LD)/2)*cos(y(3));
(r*(RD+LD)/2)*sin(y(3));
(r*(RD-LD)/L)
];
function [t,y] =main
tend=10;
rt = linspace(0,tend,25);
RD = rt.^1/2;
lt = linspace(0,tend,25);
LD = -lt.^2;
tspan = [1 tend];
ic = [0; 0; 0];
opts = odeset('RelTol',1e-2,'AbsTol',1e-4);
[t,y] = ode45(@(t,y) diffdrive(t,y,rt,RD,lt,LD), tspan, ic, opts);
% plot(y(:,1),y(:,2));grid
plot(y(:,1),y(:,2));grid
xlabel('Time (sec)','FontSize',14,'FontWeight','bold','Color','k');
ylabel('Signals','FontSize',14,'FontWeight','bold','Color','k');
title('Robot Pose Versus Time','FontSize',16,'FontWeight','bold','Color','k');