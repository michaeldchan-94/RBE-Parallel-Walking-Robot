%pod(P)
%The vectors, lengths, and unit vectors of pods can be calculated using
%this function. The argument of the function is P which is the position
%vector of the upper platform. Note that P Xp and teta which are as follows:
%Xp: is a vector from the origin to the center of the upper platform
%teta: is a vector containing the angle of the upper platform in space.
%Note that the angle of the platform must be in degrees
%Rm: is the radius of the upper platform
%Rf: is the radius of the lower platform
%alpha: is the angular distance between two universal joints. Note that the
%angle must be in radian (alpha = 40 here)
%beta: is the angular distance between two spherical joints. Note that the
%angle must be in radian (beta = 85 here)
function [L,l,n,s,u,R]=pod(P, Param)
simuParam = [92.1597 84.4488 0 305.4001 111.1565 0 604.8652;
             27.055 122.037 0 -56.4357 320.0625 0 604.8652;
           -119.2146 37.5882 0 -248.9644 208.9060 0 604.8652;
          -119.2146 -37.5882 0 -248.9644 -208.9060 0 604.8652;
             27.055 -122.037 0 -56.4357 -320.0625 0 604.8652;
            92.1597 -84.4488 0 305.4001 -111.1565 0 604.8652];

realParam = [96.6610, 81.7602, 1.0684, 305.2599, 115.0695, 2.6210, 604.4299;
    22.2476, 125.2511, -0.5530, -55.2814, 322.9819, 4.2181, 607.2473;
    -122.4519, 36.6453, 4.3547, -244.7954, 208.0087, 3.9365, 600.4441;
    -120.6859, -34.4565, -4.9014, -252.5755, -211.8783, -3.0128, 605.9031;
    24.7769, -125.0489, -4.8473, -53.9678, -320.6115, 4.3181, 604.5251;
    91.3462 -80.9866 0.2515 302.4266 -109.4351 3.3812 600.0616];

%         extract out the si and ui components from real param
siReal = realParam(:,1:3);
uiReal = realParam(:,4:6);
siNom = simuParam(:,1:3);
uiNom = simuParam(:,4:6);

siDiff = siReal-siNom;
uiDiff = uiReal-uiNom;

%        need simu and real lengths to calc deltaL
realLengths = realParam(:,7);
nomLength = simuParam(:,7);

%         calculate deltaL
deltaL = realLengths - nomLength;


switch lower(Param)
    case 'nominal'
        Rm=125;
        Rf=325;
        % Rm=250.9339/2;
        % Rf=634.2376/2;
        alpha=40*pi/180;
        beta=85*pi/180;
        Xp=P(1:3,1);
        a=P(4)*pi/180;
        b=P(5)*pi/180;
        c=P(6)*pi/180;
        %                 teta=P(4:6,1);
        R1=[1,0,0;0,cos(a),-sin(a);0,sin(a),cos(a)];
        R2=[cos(b),0,sin(b);0,1,0;-sin(b),0,cos(b)];
        R3=[cos(c),-sin(c),0;sin(c),cos(c),0;0,0,1];
        R=R1*R2*R3;
        %                 R=[cos(teta(3))*cos(teta(2)),-sin(teta(3))*cos(teta(1))+sin(teta(1))*sin(teta(2))*cos(teta(3)),sin(teta(1))*sin(teta(3))+cos(teta(1))*cos(teta(3))*sin(teta(2));
        %                     sin(teta(3))*cos(teta(2)),cos(teta(3))*cos(teta(1)),sin(teta(1))*sin(teta(2))*sin(teta(3));
        %                     -sin(teta(2)),sin(teta(1))*cos(teta(2)),cos(teta(1))*cos(teta(2))];
        %R is the rotation matrix
        s=[Rm*cos(beta/2),-Rm*sin(pi/6-beta/2),-Rm*sin(pi/6+beta/2),-Rm*cos(pi/3-beta/2),-Rm*cos(pi/3+beta/2),Rm*cos(beta/2);
            Rm*sin(beta/2),Rm*cos(pi/6-beta/2),Rm*cos(pi/6+beta/2),-Rm*sin(pi/3-beta/2),-Rm*sin(pi/3+beta/2),-Rm*sin(beta/2);
            0,0,0,0,0,0];
        %s is a matrix whose columns contain vactors of spherical joints in a
        %coordination system which is linked to the upper platform
        u=[Rf*cos(alpha/2),-Rf*sin(pi/6-alpha/2),-Rf*sin(pi/6+alpha/2),-Rf*cos(pi/3-alpha/2),-Rf*cos(pi/3+alpha/2),Rf*cos(alpha/2);
            Rf*sin(alpha/2),Rf*cos(pi/6-alpha/2),Rf*cos(pi/6+alpha/2),-Rf*sin(pi/3-alpha/2),-Rf*sin(pi/3+alpha/2),-Rf*sin(alpha/2);
            0,0,0,0,0,0];
        %u is a matrix whose columns contain vactors of universal joints in a
        %coordination system which is linked to the lower platform
        for i=1:6
            L(:,i)=R*s(:,i)+Xp-u(:,i);
            l(i)=norm(L(:,i),2);
            n(:,i)=L(:,i)/l(i);
        end
        %L is the vector of each pod and l is a vector containing the lenghths of
        %pods
        
    case 'real'
        %% input real parameters for use in real forward kin
        Rm=125;
        Rf=325;
        % Rm=250.9339/2;
        % Rf=634.2376/2;
        alpha=40*pi/180;
        beta=85*pi/180;
        Xp=P(1:3,1);
        a=P(4)*pi/180;
        b=P(5)*pi/180;
        c=P(6)*pi/180;
        %         teta=P(4:6,1);
        R1=[1,0,0;0,cos(a),-sin(a);0,sin(a),cos(a)];
        R2=[cos(b),0,sin(b);0,1,0;-sin(b),0,cos(b)];
        R3=[cos(c),-sin(c),0;sin(c),cos(c),0;0,0,1];
        R=R1*R2*R3;
        %                 R=[cos(teta(3))*cos(teta(2)),-sin(teta(3))*cos(teta(1))+sin(teta(1))*sin(teta(2))*cos(teta(3)),sin(teta(1))*sin(teta(3))+cos(teta(1))*cos(teta(3))*sin(teta(2));
        %                     sin(teta(3))*cos(teta(2)),cos(teta(3))*cos(teta(1)),sin(teta(1))*sin(teta(2))*sin(teta(3));
        %                     -sin(teta(2)),sin(teta(1))*cos(teta(2)),cos(teta(1))*cos(teta(2))];
        %R is the rotation matrix
        s=[Rm*cos(beta/2),-Rm*sin(pi/6-beta/2),-Rm*sin(pi/6+beta/2),-Rm*cos(pi/3-beta/2),-Rm*cos(pi/3+beta/2),Rm*cos(beta/2);
            Rm*sin(beta/2),Rm*cos(pi/6-beta/2),Rm*cos(pi/6+beta/2),-Rm*sin(pi/3-beta/2),-Rm*sin(pi/3+beta/2),-Rm*sin(beta/2);
            0,0,0,0,0,0];
        s = s+siDiff';
        %s is a matrix whose columns contain vactors of spherical joints in a
        %coordination system which is linked to the upper platform
        u=[Rf*cos(alpha/2),-Rf*sin(pi/6-alpha/2),-Rf*sin(pi/6+alpha/2),-Rf*cos(pi/3-alpha/2),-Rf*cos(pi/3+alpha/2),Rf*cos(alpha/2);
            Rf*sin(alpha/2),Rf*cos(pi/6-alpha/2),Rf*cos(pi/6+alpha/2),-Rf*sin(pi/3-alpha/2),-Rf*sin(pi/3+alpha/2),-Rf*sin(alpha/2);
            0,0,0,0,0,0];
        u = u+uiDiff';
        %u is a matrix whose columns contain vactors of universal joints in a
        %coordination system which is linked to the lower platform
        
        %         apply deltaL
        for j=1:6
            L(:,j)=R*s(:,j)+Xp-u(:,j);
            l(j)=norm(L(:,j),2)-deltaL(j);
            n(:,j)=L(:,j)/l(j);
        end
        %L is the vector of each pod and l is a vector containing the lenghths of
        %pods
        
end