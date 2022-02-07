%% Question 3

t = [1:11];
beta = linspace(0.5,1,11);
figure;
for i = 1:11
    betaP(i) = (beta(i)/(1-beta(i)));
    fplot(@(x) betaP(i)*x);
    hold on
    xlim([0 10])
    ylim([0 100])
end
title('Graph of u(t) = B(t)/1-B(t)*V(t) with varying values for B');
xlabel('v(t)');
ylabel('u(t)');


hold off;
%% Question 4
% We know that the robot has a stride of 0.16m. We know the initial
% conditions and the end conditions that we need to meet. So calculating for position and
% velocity is trivial.

t = [0:0.01:1];




%% Horizontal position and Velocity

% conditions to be met
% x0 = 0
% xf = 0.16
% x0dot = 0
% xfdot = 0

syms a b c d
eq1 = a+b+c+d == 0.16; %xf condition (x = 0.16)
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

%% Vertical position and velocity 

syms omega x xdot
A = 1;
xf = 0.16; %m/s
% y = A*sin(omega*x);
% at end of period, omega*x should = pi.

% Find omega
eq1 = omega*xf == pi;
omSol = solve(eq1,omega);
omega = double(omSol);

% set up equations for vertical position and velocity
y = sin(omega*x);
ydot = xdot * diff(y,x);
%% Evaluation over time
j = 1;
for i = [0:0.01:1]
    xeval(j) = polyval(xeq,i);
    xdoteval(j) = polyval(xdoteq,i);
    yeval(j) = (subs(y,x,xeval(j)));
    ydoteval(j) = (subs(ydot,[xdot,x],[xdoteval(j),xeval(j)]));
%   For respect to body, just take xeval and subtract body velocity at time
%   t
    xbody(j) = xeval(j)-0.04*i;
    j = j+1;
end

% change to doubles for plotting
yeval = double(yeval);
ydoteval = double(ydoteval);

%% Plotting
figure('units','normalized','outerposition',[0 0 1 1])
hold on
subplot(3,2,1)
plot(t,xdoteval)
ylabel('Horizontal Velocity m/s')
xlabel('Time')
subplot(3,2,2)
plot(t,ydoteval)
ylabel('Vertical Velocity m/s')
xlabel('Time')
subplot(3,2,3)
plot(t,xeval)
ylabel('Horizontal Position')
xlabel('Time')
subplot(3,2,4)
plot(t,yeval)
ylabel('Vertical Position')
xlabel('Time')
subplot(3,2,5)
plot(xeval,yeval)
ylabel('Vertical Position')
xlabel('Horizontal Position')
subplot(3,2,6)
plot(xbody,yeval)
ylabel('Vertical Position')
xlabel('Horizontal Position wrt Body')
hold off;




