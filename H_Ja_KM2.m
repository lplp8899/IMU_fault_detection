%----------------------------------------------------------------------
%   Author: Peng Lu, Delft University of Technology 
%   email:  P.Lu-1@tudelft.nl
%   released in May 2016
%----------------------------------------------------------------------

function H=H_Ja_KM2(x)

uGS_b=x(1,:);vGS_b=x(2,:);wGS_b=x(3,:);
phi=x(4,:);theta=x(5,:);psi=x(6,:);

H=[cos(psi)*cos(theta),cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi),cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi),cos(psi)*sin(theta)*(cos(phi)*vGS_b-sin(phi)*wGS_b)-sin(psi)*(-sin(phi)*vGS_b-cos(phi)*wGS_b),cos(psi)*(-sin(theta)*uGS_b+cos(theta)*(sin(phi)*vGS_b+cos(phi)*wGS_b)),-sin(psi)*(cos(theta)*uGS_b+sin(theta)*(sin(phi)*vGS_b+cos(phi)*wGS_b))-cos(psi)*(cos(phi)*vGS_b-sin(phi)*wGS_b);
sin(psi)*cos(theta),sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi),sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi),sin(psi)*sin(theta)*(cos(phi)*vGS_b-sin(phi)*wGS_b)+cos(psi)*(-sin(phi)*vGS_b-cos(phi)*wGS_b),sin(psi)*(-sin(theta)*uGS_b+cos(theta)*(sin(phi)*vGS_b+cos(phi)*wGS_b)),cos(psi)*(cos(theta)*uGS_b+sin(theta)*(sin(phi)*vGS_b+cos(phi)*wGS_b))-sin(psi)*(cos(phi)*vGS_b-sin(phi)*wGS_b);
-sin(theta),cos(theta)*sin(phi),cos(theta)*cos(phi),cos(theta)*(cos(phi)*vGS_b-sin(phi)*wGS_b),-cos(theta)*uGS_b-sin(theta)*(sin(phi)*vGS_b+cos(phi)*wGS_b),0;
0,0,0,1,0,0;
0,0,0,0,1,0;
0,0,0,0,0,1;];
