% Chan
function Chan_HW4()
l1=.030;
l2=.050;
l3=.10;
b = 0.5;
bodyVel = 0.1;
h=.1; %(m) height of the robot.
D=l1+l2; %(m) Distance between the foot tip and hip joint from top view
R=.220; % distance of foot tip to COG or center of rotation
u=(b/(1-b))*bodyVel; % average swing velocity of the leg wrt the body
u=bodyVel/(1-b); % average swing velocity of the leg wrt the ground
a=pi/6; % Crab Angle
L=.16; % Stride Length which is the amount of COG displacement in one cycle time.
T= L/bodyVel; % Cycle Period
Tt=(1-b)*T;

%% Gait Planner
% This code has been adapted from the example code given by Professor
% Agheli

% segment into partitions 6 equal timesteps according to example
time = linspace(0,Tt,6);

xdotmaxf_g=2*(time(6)-time(1))*u/((time(5)-time(2))+(time(4)-time(3)));

for i = 1:6
xPositions(i,:) = [-L/2;-L/2;-L/4;L/4;L/2;L/2];
zPositions(i,:) = [0;h/2;h;h;h/2;0];
end

xdot = [0;0;xdotmaxf_g;xdotmaxf_g;0;0];
zdot = [0;xdotmaxf_g;0;0;-xdotmaxf_g;0];
alphaH=[-30*pi/180, 30*pi/180, -90*pi/180, 90*pi/180, -150*pi/180, 150*pi/180];


% % xPositions_g = xPositions + L/2;
xb_g(1,1)=-((1-b)/2)*L-D*cos(alphaH(1)); % frame b is a frame attached to the hip joint whose
xb_g(2,1)=-((1-b)/2)*L-D*cos(alphaH(2));
xb_g(3,1)=-((1-b)/2)*L;
xb_g(4,1)=-((1-b)/2)*L;
xb_g(5,1)=-((1-b)/2)*L-D*cos(alphaH(5));
xb_g(6,1)=-((1-b)/2)*L-D*cos(alphaH(6));

% x axis is parallel to the x axis of frame g (Gi).
zb_g(1,1)=h;
zb_g(2,1)=h;
zb_g(3,1)=h;
zb_g(4,1)=h;
zb_g(5,1)=h;
zb_g(6,1)=h;

dt=Tt/5;
xdotb_g=bodyVel;
zdotb_g=0;

for i=1:6
    for t=1:5 % 5 is becaus of time dividing to 5 equal extents.
        xb_g(i,t+1)=xb_g(i,t)+xdotb_g*dt;
        zb_g(i,t+1)=zb_g(i,t)+zdotb_g*dt;
    end
    for t=1:6
        xf_b(i,t)=xPositions(i,t)-xb_g(i,t);
        zf_b(i,t)=zPositions(i,t)-zb_g(i,t);
    end
end
% ****xf_b(t) and zf_b(t) ARE THE SAME FOR ALL LEGS i=1,2,...,6. But yf_b
% differs as follow. For each leg, yf_b is the same for all t. 
yf_b=[D*sin(30*pi/180), -D*sin(30*pi/180), D,  -D,  D*sin(30*pi/180),  -D*sin(30*pi/180)];

for i=1:6 % this is becaus of 6 legs
    for j=1:6 % this is because we have divided out T to 6 dt.
              %xf_H(i,j)=RH_b*xf_b(i,j);
    
        xf_H(i,j)=[cos(alphaH(i)),-sin(alphaH(i)),0]*[xf_b(i,j);yf_b(i);zf_b(i,j)];
        yf_H(i,j)=[sin(alphaH(i)),cos(alphaH(i)),0]*[xf_b(i,j);yf_b(i);zf_b(i,j)];
        zf_H(i,j)=zf_b(i,j);
    end
end
makePosnVelPlot(time,xdot,zdot,xPositions,zPositions,xf_b,zf_b)

%% IK
for i=1:6 % for all 6 legs
    for j=1:6 % time discrete
        Alpha(i,j)=(atan(yf_H(i,j)/xf_H(i,j)));
        l(i,j)=sqrt(yf_H(i,j)^2+xf_H(i,j)^2);
        d(i,j)=sqrt(zf_H(i,j)^2+(l(i,j)-l1)^2);
        Beta(i,j)=acos((l2^2+d(i,j)^2-l3^2)/(2*l2*d(i,j)))-atan(abs(zf_H(i,j))/(l(i,j)-l1));
        Gamma(i,j)=pi-(acos((l2^2+l3^2-d(i,j)^2)/(2*l2*l3)));
%         Beta(i,j)=Beta(i,j)*180/pi;
%         Gamma(i,j)=pi-Gamma(i,j);
    end
end
A=Alpha*180/pi
B=Beta*180/pi
G=Gamma*180/pi

%% 
ydotf_b=0;
for t=1:6
    xdotf_b(t)=xdot(t)-xdotb_g;
    zdotf_b(t)=zdot(t)-zdotb_g;
end
for i=1:6
    for j=1:6
        xdotf_H(i,j)=[cos(alphaH(i)),-sin(alphaH(i)),0]*[xdotf_b(j);ydotf_b;zdotf_b(j)];
        ydotf_H(i,j)=[sin(alphaH(i)),cos(alphaH(i)),0]*[xdotf_b(j);ydotf_b;zdotf_b(j)];
        zdotf_H(i,j)=zdotf_b(j);
    end
end
for j=1:6 % time
    for i=1:6 % leg numnber
        theta1=Alpha(i,j);
        theta2=Beta(i,j);
        theta3=Gamma(i,j);
        J(1,1)=-(-sin(theta1)*sin(theta2)*cos(theta3)-sin(theta1)*cos(theta2)*sin(theta3))*l3-sin(theta1)*l2*cos(theta2)-l1*sin(theta1);
        J(1,2)=-(-cos(theta1)*sin(theta2)*sin(theta3)+cos(theta1)*cos(theta2)*cos(theta3))*l3-cos(theta1)*l2*sin(theta2);
        J(1,3)=(cos(theta1)*sin(theta2)*sin(theta3)-cos(theta1)*cos(theta2)*cos(theta3))*l3;
        J(2,1)=-(cos(theta1)*cos(theta2)*sin(theta3)+cos(theta1)*sin(theta2)*cos(theta3))*l3+cos(theta1)*l2*cos(theta2)+l1*cos(theta1);
        J(2,2)=-(-sin(theta1)*sin(theta2)*sin(theta3)+sin(theta1)*cos(theta2)*cos(theta3))*l3-sin(theta1)*l2*sin(theta2);
        J(2,3)=-(-sin(theta1)*sin(theta2)*sin(theta3)+sin(theta1)*cos(theta2)*cos(theta3))*l3;
        J(3,1)=0;
        J(3,2)=-(-cos(theta2)*sin(theta3)-sin(theta2)*cos(theta3))*l3-l2*cos(theta2);
        J(3,3)=-(-cos(theta2)*sin(theta3)-sin(theta2)*cos(theta3))*l3;
        Thetadot(i,j,:)=inv(J)*[xdotf_H(i,j);ydotf_H(i,j);zdotf_H(i,j)];
    end
end

Thetadot
end



function makePosnVelPlot(t,xdoteval,zdoteval,xeval,zeval,xbody,zbody)
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
plot(t,xeval(4,:))
ylabel('Horizontal Position')
xlabel('Time')
subplot(3,2,4)
plot(t,zeval(4,:))
ylabel('Vertical Position')
xlabel('Time')
subplot(3,2,5)
plot(xeval(4,:),zeval(4,:))
ylabel('Vertical Position')
xlabel('Horizontal Position')
subplot(3,2,6)
plot(xbody(4,:),zbody(4,:))
ylabel('Vertical Position')
xlabel('Horizontal Position wrt Body')
hold off;
end

