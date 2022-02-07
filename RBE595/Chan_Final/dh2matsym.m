%Michael Chan
% Symbolically solves for the Homogenous equations
% doesn't use cosd and sind
function [ H ] = dh2matsym( theta, d, a, alpha )
rotz = [cosd(theta), -sind(theta),0,0;
        sind(theta),cosd(theta),0,0;
        0,0,1,0;
        0,0,0,1];
transz = [1,0,0,0;
          0,1,0,0;
          0,0,1,d;
          0,0,0,1];
rotx = [1,0,0,0;
          0,cosd(alpha),-sind(alpha),0;
          0,sind(alpha), cosd(alpha),0;
          0,0,0,1];

transx = [1,0,0,a;
          0,1,0,0;
          0,0,1,0;
          0,0,0,1];

H = rotz * transz * transx * rotx;

end