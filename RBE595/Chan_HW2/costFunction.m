function CF = costFunction(initialGuess)

%For calculating the cost function, pick +10 configurations close to the
%boundary of the robot workspace. Remember, the code works for any set of
%configurations you choose. But to get an acceptable calibration result,
%you should choose them as close to the boundary as possible.
%Anyways, these configurations are nominal configurations.

% Calculate the real configurations by using the real FK code
% (FK code in which nominal parameters are replaced with real parameters).
% Note that you would need to use IK code as well in this part to get the
% leg lengths as input to the real FK.

% Calculate the cost function using the equation available in the paper.

% real and simulated parameters
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

% extracts info from guesses
si = initialGuess(:,1:3);
ui = initialGuess(:,4:6);
simuLen = initialGuess(:,7);

% testing purposes
% si = simuParam(:,1:3)';
% ui = simuParam(:,4:6)';
% simuLen = simuParam(:,7);

% finds real leg lenths and simulated leg lengths from tables
realLen = realParam(:,7);
homePos = simuParam(:,7);

%configurations
m = [.1, -.1, 100, -100, 170, 170, -150,  150,0,80;
    .1, -.1, 100, 100, 170,  170,  150, -150, 0,-200;
    1000, 1000,800, 500,1000, 1000, 800, 1000,700,800;
    0.5,  0 ,  0,   0,   0,  .5,    0, -.5, 2, 10;
    0.5,  0,   0,   0,   0,  .5,   .5,   0, 0, 0;
    0.5,  0,   0,   0,    0,    0,    0,   0, 0, 10];

% 80,-200-800-10 0 10
n = size(m,2);
lmin = simuLen;
lmax = 1100; %mm

j = 0;
for c = 1:n
    % nominal inverse kinematics of the robot
    %     calculate the nominal leg lengths at position
    [~,fkLen,~,~,~,~] = pod(m(:,c),'nominal');
    % Find real TCPs through forward kinematics using real parameters (table 2)
    % and the leg lengths found
    pR(:,c) = ForwardKinematics(m(:,c),fkLen','real');
end

for i = 1:n
    %     find parameters of real and nominal config
    [~,ln,~,~,~,~] = pod(m(:,i),'nominal');
%     [Ln,ln,nn,sn,un,Rn] = pod(m(:,i),'nominal');
    [~,~,~,~,~,Rr] = pod(pR(:,i),'real');
    
    for k = 1:6
        
        j = j+1;
        %     nominal parameters
        
        % this code gives same readings as ln
        %     Li = m(1:3,k) + Rn*(sn(:,k))-un(:,k);
        %     li = norm(Li,2);
        
        % can calculate this straight from nominal Pod
        sqNormN = ln(k)^2;
        
        
        %     real parameters
        O = pR(1:3,i);
        %         Lr = O + Rr*(si(1:3,k))-ui(1:3,k);
        Lr = O + Rr*(si(k,1:3)')-ui(k,1:3)';
        lreal = norm(Lr,2);
        sqNormR = (homePos(k)+lreal-simuLen(k))^2;
        % equation 10
        CF(j) = (sqNormN - sqNormR)^2;
        %CF(j) = ((ln(k)^2)-(homePos(k)+norm(pR(1:3,k) + Rr*initialGuess(k,1:3)'-initialGuess(k,4:6)')-initialGuess(k,7))^2)^2;
        
    end
end
end

