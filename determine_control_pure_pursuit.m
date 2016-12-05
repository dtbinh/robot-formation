%% Ben Cannon
% Pure Pursuit
function [UR, UL] = determine_control_pure_pursuit(t, z, target_pose, L, ic)
velocity = 5;

% Name coordinates just so its easier to write
xr = ic(1); yr = ic(2); theta = ic(3);
%Goal Point
xg = target_pose(1); yg = target_pose(2); thetafinal = target_pose(3);

% Convert to vehicle frame coordinates (Different from paper ive found for
% theta from x axis)
xgv = (xg - xr)*sin(theta) - (yg-yr)*cos(theta);
ygv = (xg - xr)*cos(theta) + (yg-yr)*sin(theta);

% Distance from vehicle 
D = sqrt(xgv^2 + ygv^2);

%gamma equals 1/radius and is the curvature of the circle;
gamma = 2*(abs(xgv))/(D^2);
r = 1/gamma;
rFar = r+L/2;
rNear = r-L/2;

% Determine UL and UR
UL = velocity;
if(xg-xr==0)
    UR = UL;
else
    goal_angle = atan((yg-yr)/(xg-xr));
    if(theta<goal_angle)
        UR = (rFar/rNear)*UL;
    else
        UR = velocity;
        UL = (rFar/rNear)*UR;
    end
end



end
