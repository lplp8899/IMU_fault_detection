%----------------------------------------------------------------------
%   Author: Peng Lu, Delft University of Technology 
%   email:  P.Lu-1@tudelft.nl
%   released in May 2016
%----------------------------------------------------------------------
function G_n=G_Ja_KM2(x)



uGS_b=x(1);vGS_b=x(2);wGS_b=x(3);
phi_n=x(4);theta_n=x(5);%psi=x(9);



    G_n=[ -1  0  0     0            wGS_b                   -vGS_b;...
      0  -1  0    -wGS_b         0                       uGS_b;...
      0  0  -1    vGS_b        -uGS_b                     0  ;...
      0  0  0     -1   -sin(phi_n)*tan(theta_n)    -cos(phi_n)*tan(theta_n);...
      0  0  0      0         -cos(phi_n)               sin(phi_n) ;...
      0  0  0      0   -sin(phi_n)/cos(theta_n)    -cos(phi_n)/cos(theta_n); ];
