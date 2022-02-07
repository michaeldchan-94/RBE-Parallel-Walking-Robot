%This function solves the direct kinematics of hexapod mechanism using
%newton-Raphson method. The function inputs the initial guess (P0)
%(position of the upper platform) together with a vector containing pod's
%lengths (lg)i.e. [l1;l2;l3;l4;l5], respectively and outputs the
%approximation of the upper platform's position no farther than the
%specified precision. The precision is considered to be 0.0001 mm.
function p=ForwardKinematics(P0,lg,Param)
switch lower (Param)
    case 'nominal'
        t=1;
        Rm=125;
        Rf=325;
        % Rm=250.9339/2;
        % Rf=634.2376/2;
        alpha=40*pi/180;
        beta=85*pi/180;
        P(:,1)=P0;
        i=2;
        while t>0.000001
            X=P(1:3,i-1);
            teta=P(4:6,i-1);
            J=jacobianV(P(:,i-1),'nominal');
            [L,l,n]=pod(P(:,i-1),'nominal');
            Rp=[1,0,0,0,0,0;
                0,1,0,0,0,0;
                0,0,1,0,0,0;
                0,0,0,1,0,sin(teta(2)*pi/180);
                0,0,0,0,cos(teta(1)*pi/180),-sin(teta(1)*pi/180)*cos(teta(2)*pi/180);
                0,0,0,0,sin(teta(1)*pi/180),cos(teta(1)*pi/180)*cos(teta(2)*pi/180)];
            JRp=J*Rp;
            % JRp=J;
            P(:,i)=P(:,i-1)-inv(JRp)*(l'-lg);
            t=norm(P(:,i)-P(:,i-1),2);
            i=i+1;
        end
        p=P(:,end);
        
    case 'real'
        t=1;
        Rm=125;
        Rf=325;
        % Rm=250.9339/2;
        % Rf=634.2376/2;
        alpha=40*pi/180;
        beta=85*pi/180;
        P(:,1)=P0;
        i=2;
        while t>0.000001
            X=P(1:3,i-1);
            teta=P(4:6,i-1);
            J=jacobianV(P(:,i-1),'real');
            [L,l,n]=pod(P(:,i-1),'real');
            Rp=[1,0,0,0,0,0;
                0,1,0,0,0,0;
                0,0,1,0,0,0;
                0,0,0,1,0,sin(teta(2)*pi/180);
                0,0,0,0,cos(teta(1)*pi/180),-sin(teta(1)*pi/180)*cos(teta(2)*pi/180);
                0,0,0,0,sin(teta(1)*pi/180),cos(teta(1)*pi/180)*cos(teta(2)*pi/180)];
            JRp=J*Rp;
            % JRp=J;
            P(:,i)=P(:,i-1)-inv(JRp)*(l'-lg);
            t=norm(P(:,i)-P(:,i-1),2);
            i=i+1;
        end
        p=P(:,end);
end
        