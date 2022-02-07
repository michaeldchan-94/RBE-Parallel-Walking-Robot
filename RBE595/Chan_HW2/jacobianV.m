%JacobianV(Xp,teta,Rm,Rf,alpha,beta)
%This function determines the Jacobi matrix of a hexapod mechanism by
%inputing the position vector of the upper platform.
%The argument of the function is P which is the position
%vector of the upper platform. Note that P Xp and teta which are as follows:
%Xp: is a vector from the origin to the center of the upper platform
%teta: is a vector containing the angle of the upper platform in space.
%Note that the angle of the platform must be in radian
%Rm: is the radius of the upper platform (Rm = 125 here)
%Rf: is the radius of the lower platform (Rf = 325 here)
%alpha: is the angular distance between two spherical joints. Note that the
%angle must be in radian (alpha = 40 here)
%beta: is the angular distance between two universal joints. Note that the
%angle must be in radian (beta = 85 here)
function J=jacobianV(P,Param)
switch lower (Param)
    case 'nominal'
        Rm=125;
        Rf=325;
        % Rm=250.9339/2;
        % Rf=634.2376/2;
        alpha=40*pi/180;
        beta=85*pi/180;
        Xp=P(1:3,1);
        teta(1)=P(4)*pi/180;
        teta(2)=P(5)*pi/180;
        teta(3)=P(6)*pi/180;
        [L,l,n,s,u,R]=pod(P,'nominal');
        % R=[cos(teta(3))*cos(teta(2)),-sin(teta(3))*cos(teta(1))+sin(teta(1))*sin(teta(2))*cos(teta(3)),sin(teta(1))*sin(teta(3))+cos(teta(1))*cos(teta(3))*sin(teta(2));
        %     sin(teta(3))*cos(teta(2)),cos(teta(3))*cos(teta(1)),sin(teta(1))*sin(teta(2))*sin(teta(3));
        %     -sin(teta(2)),sin(teta(1))*cos(teta(2)),cos(teta(1))*cos(teta(2))];
        % s=[Rm*cos(beta/2),-Rm*sin(pi/6-beta/2),-Rm*sin(pi/6+beta/2),-Rm*cos(pi/3-beta/2),-Rm*cos(pi/3+beta/2),Rm*cos(beta/2);
        %     Rm*sin(beta/2),Rm*cos(pi/6-beta/2),Rm*cos(pi/6+beta/2),-Rm*sin(pi/3-beta/2),-Rm*sin(pi/3+beta/2),-Rm*sin(beta/2);
        %     0,0,0,0,0,0];
        J=[n(:,1)',cross(R*s(:,1),n(:,1))';
            n(:,2)',cross(R*s(:,2),n(:,2))';
            n(:,3)',cross(R*s(:,3),n(:,3))';
            n(:,4)',cross(R*s(:,4),n(:,4))';
            n(:,5)',cross(R*s(:,5),n(:,5))';
            n(:,6)',cross(R*s(:,6),n(:,6))'];
    case 'real'
        Rm=125;
        Rf=325;
        % Rm=250.9339/2;
        % Rf=634.2376/2;
        alpha=40*pi/180;
        beta=85*pi/180;
        Xp=P(1:3,1);
        teta(1)=P(4)*pi/180;
        teta(2)=P(5)*pi/180;
        teta(3)=P(6)*pi/180;
        [L,l,n,s,u,R]=pod(P,'real');
        % R=[cos(teta(3))*cos(teta(2)),-sin(teta(3))*cos(teta(1))+sin(teta(1))*sin(teta(2))*cos(teta(3)),sin(teta(1))*sin(teta(3))+cos(teta(1))*cos(teta(3))*sin(teta(2));
        %     sin(teta(3))*cos(teta(2)),cos(teta(3))*cos(teta(1)),sin(teta(1))*sin(teta(2))*sin(teta(3));
        %     -sin(teta(2)),sin(teta(1))*cos(teta(2)),cos(teta(1))*cos(teta(2))];
        % s=[Rm*cos(beta/2),-Rm*sin(pi/6-beta/2),-Rm*sin(pi/6+beta/2),-Rm*cos(pi/3-beta/2),-Rm*cos(pi/3+beta/2),Rm*cos(beta/2);
        %     Rm*sin(beta/2),Rm*cos(pi/6-beta/2),Rm*cos(pi/6+beta/2),-Rm*sin(pi/3-beta/2),-Rm*sin(pi/3+beta/2),-Rm*sin(beta/2);
        %     0,0,0,0,0,0];
        J=[n(:,1)',cross(R*s(:,1),n(:,1))';
            n(:,2)',cross(R*s(:,2),n(:,2))';
            n(:,3)',cross(R*s(:,3),n(:,3))';
            n(:,4)',cross(R*s(:,4),n(:,4))';
            n(:,5)',cross(R*s(:,5),n(:,5))';
            n(:,6)',cross(R*s(:,6),n(:,6))'];
end