% Chan
function Chan_HW4()
% need Gait planning (hw3) done
% need foot planning lecture 9 TODO
% Need all angle positions and velocities and graphs TODO
% vertical lift up and touch down (square wave) done
l1=.020;
l2=.050;
l3=.100;
strideLength = 0.16;
b = 0.5;
footVel = 0.1;
bodyVel = 0.1;
h=.1; %(m) height of the robot. Here h=l3 because it is assumed in the home positon.
D=l1+l2; %(m) Distance between the foot tip and hip joint from top view
R=.220; % distance of foot tip to COG or center of rotation
u=(b/(1-b))*bodyVel; % average swing velocity of the leg wrt the body
u=bodyVel/(1-b); % average swing velocity of the leg wrt the ground
a=pi/6; % Crab Angle
L=.16; % Stride Length which is the amount of COG displacement in one cycle time.
T= L/bodyVel % Cycle Period
Tt=(1-b)*T;
Ts=b*T;

%% Gait Planner
[xeq, xdoteq] = computeHorizontal(strideLength);
j = 1;
t = [0:0.01:1];
for i = [0:0.01:1]
    xeval(j) = polyval(xeq,i);
    xdoteval(j) = polyval(xdoteq,i);
    zeval(j) = computeZ(j);
    zdoteval(j) = computeZdot(j,b,bodyVel);
    xbody(j) = xeval(j)-bodyVel*i;
    j = j+1;
end
makePosnVelPlot(t,xdoteval,zdoteval,xeval,zeval,xbody);

%% Foot Trajectory Planning
%%TODO

end

function [xeq,xdoteq] = computeHorizontal(strideLength)
%% Helper function to compute horizontal position and Velocity

% conditions to be met
% x0 = 0
% xf = 0.16
% x0dot = 0
% xfdot = 0

syms a b c d
eq1 = a+b+c+d ==strideLength; %xf condition
eq2 = 3*a+2*b+c == 0; %xfdot condition (xdot = 0)
eq3 = d == 0; %initial x conditions (x = 0)
eq4 = c == 0; %initial xdot conditions (xdot = 0)
sol = solve([eq1, eq2, eq3, eq4], [a,b,c,d]);
a = double(sol.a);
b = double(sol.b);
c = double(sol.c);
d = double(sol.d);

% Set up equations for horizontal position and velocity
xeq = [a,b,c,d];
xdoteq = [3*a,2*b,c];
end

function zdot = computeZdot(j,beta,bodyVel)
%% Helper function to calculate Y velocity
% This function takes in the counter j and outputs the appropriate velocity
% at that point in time.
footVel = (beta/(1-beta))*bodyVel;

if j == 20
    zdot = footVel;
elseif j == 80
    zdot = -footVel;
else
    zdot = 0;
end
end

function [zpos] =  computeZ(j)
%% Vertical position and velocity helper function
% This function takes in the current xpos and generates a square wave based
% on what the xposition is. If it is greater than the 20th step, set high,
% if after the 80th step, set low.

% conditions to be met
% y0 = 0
% y(0.1) = 1
% y(0.8) = 0

if ((20<=j)&&(j<=80))
    zpos = 1;
else
    zpos = 0;
end
end


function makePosnVelPlot(t,xdoteval,zdoteval,xeval,zeval,xbody)
%% Plotting
figure('units','normalized','outerposition',[0 0 1 1])
hold on
subplot(3,2,1)
plot(t,xdoteval)
ylabel('Horizontal Velocity m/s')
xlabel('Time')
subplot(3,2,2)
plot(t,zdoteval)
ylabel('Vertical Velocity m/s')
xlabel('Time')
subplot(3,2,3)
plot(t,xeval)
ylabel('Horizontal Position')
xlabel('Time')
subplot(3,2,4)
plot(t,zeval)
ylabel('Vertical Position')
xlabel('Time')
subplot(3,2,5)
plot(xeval,zeval)
ylabel('Vertical Position')
xlabel('Horizontal Position')
subplot(3,2,6)
plot(xbody,zeval)
ylabel('Vertical Position')
xlabel('Horizontal Position wrt Body')
hold off;
end

