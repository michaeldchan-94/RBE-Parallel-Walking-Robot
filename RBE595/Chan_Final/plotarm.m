% Chan
function [ H,legPlot ] = plotarm( q1,q2,q3,l1,l2,l3,legNumber,robotHeight,robotLength,robotWidth )

%Takes in joint angles and outputs a 3d plot of the arm.
%Can plot over time to show arm moving in space
% Adapted from 6 DoF arm to 3 for legs of hexapod

% % zero position
% theta1 = 0;
% theta2 = 0;
% theta3 = 0;
% theta4 = 0;
% theta5 = 0;
% theta6 = 0;

% assigns inputs as thetas
theta1 = q1;
theta2 = q2;
theta3 = q3;
% theta4 = q4;
% theta5 = q5;
% theta6 = q6;

% Forward Kinematics
% mm -> cm = mm*10
%theta,d,a,alpha
% H1 = dh2mat(theta1,29,0,-90);
% H2 = dh2mat(-90+theta2,0,27,0);
% HF1 = H1 * H2;
% H3 = dh2mat(180+theta3,0,-7,90);
% HF2 = H1*H2*H3;
% H4 = dh2mat(theta4,30.2,0,-90);
% HF3 = H1*H2*H3*H4;
% H5 = dh2mat(theta5,0,0,90);
% HF4 = H1*H2*H3*H4*H5;
% H6 = dh2mat(theta6,7.2,0,0);
% HF = H1 * H2 * H3 * H4 * H5 * H6

if legNumber == 1
    H0 = calcLegHome(-robotWidth/2,robotLength/2,robotHeight);
    %     Starting point for leg 1
elseif legNumber ==2
    H0 = calcLegHome(robotWidth/2,robotLength/2,robotHeight);
    %     Starting point for leg 2
elseif legNumber ==3
    H0 = calcLegHome(-robotWidth/2,0,robotHeight);
    %     Starting point for leg 3
elseif legNumber ==4
    H0 = calcLegHome(robotWidth/2,0,robotHeight);
    %     Starting point for leg 4
elseif legNumber ==5
    H0 = calcLegHome(-robotWidth/2,-robotLength/2,robotHeight);
    %     Starting point for leg 5
elseif legNumber ==6
    H0 = calcLegHome(robotWidth/2,-robotLength/2,robotHeight);
    %     Starting point for leg 6
end

% Homogenous equations for legs
H1 = dh2matsym(q1, 0, l1, -pi/2);
HF0 = H0*H1;
H2 = dh2matsym(q2, 0, l2, 0);
HF1 = H0*H1*H2;
H3 = dh2matsym(q3, 0, l3, 0);
HF2 = H0*H1*H2*H3;
H4 = dh2matsym(pi/2, 0, 0, pi/2);
HF3 = H0*H1*H2*H3*H4;
H = HF3;




% PLOTTING PART
% figure

% clf
% hold on;
% axis([-50 50 -50 50 0 100]);
% view(-15,25);


p0 = [0,0,robotHeight];
if legNumber ==1||legNumber == 3|| legNumber==5
    p1 = [-H0(13)-robotWidth,H0(14),H0(15)];
    p2 = [-HF0(13)-robotWidth,HF0(14),HF0(15)];
    p3 = [-HF1(13)-robotWidth,HF1(14),HF1(15)];
    p4 = [-HF2(13)-robotWidth,HF2(14),HF2(15)];
    p5 = [-HF3(13)-robotWidth,HF3(14),HF3(15)];
else
    % creates points to plot in plot3
    p1 = [H0(13),H0(14),H0(15)];
    % p2 = [H1(13),H1(14),H1(15)];
    p2 = [HF0(13),HF0(14),HF0(15)];
    p3 = [HF1(13),HF1(14),HF1(15)];
    p4 = [HF2(13),HF2(14),HF2(15)];
    p5 = [HF3(13),HF3(14),HF3(15)];
    % p6 = [HF4(13),HF4(14),HF4(15)];
    % p7 = [HF(13),HF(14),HF(15)];
end

% plots points
legPlot(1) = plot3(p0(1),p0(2),p0(3),'r.','MarkerSize',20);
legPlot(2) = plot3(p1(1),p1(2),p1(3),'r.','MarkerSize',20);
legPlot(3) =plot3(p2(1),p2(2),p2(3),'r.','MarkerSize',20);
legPlot(4) =plot3(p3(1),p3(2),p3(3),'r.','MarkerSize',20);
legPlot(5) =plot3(p4(1),p4(2),p4(3),'r.','MarkerSize',20);
legPlot(6) =plot3(p5(1),p5(2),p5(3),'r.','MarkerSize',20);
% plot3(p6(1),p6(2),p6(3),'r.','MarkerSize',20);
% plot3(p7(1),p7(2),p7(3),'r.','MarkerSize',20);

% splices matrices for lines
p01 = [p1;p0];
p12 = [p2;p1];
p23 = [p3;p2];
p34 = [p4;p3];
p45 = [p5;p4];
% p56 = [p6;p5];
% p67 = [p7;p6];


% plots points
legPlot(7) = plot3(p01(:,1),p01(:,2),p01(:,3),'r');
legPlot(8) = plot3(p12(:,1),p12(:,2),p12(:,3),'r');
legPlot(9) = plot3(p23(:,1),p23(:,2),p23(:,3),'r');
legPlot(10) = plot3(p34(:,1),p34(:,2),p34(:,3),'r');
legPlot(11) = plot3(p45(:,1),p45(:,2),p45(:,3),'r');
% plot3(p56(:,1),p56(:,2),p56(:,3),'r');
% plot3(p67(:,1),p67(:,2),p67(:,3),'r');
% hold off;
% drawnow;

end

function H0 = calcLegHome(x,y,z)
Tx = [1,0,0,x;
    0,1,0,0;
    0,0,1,0;
    0,0,0,1];
Ty = [1,0,0,0;
    0,1,0,y;
    0,0,1,0;
    0,0,0,1];
Tz = [1,0,0,0;
    0,1,0,0;
    0,0,1,z;
    0,0,0,1];
H0 = Tx*Ty*Tz;

end


