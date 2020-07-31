%----------------------------------------------------------------------
%   Author: Peng Lu, Delft University of Technology 
%   email:  P.Lu-1@tudelft.nl
%   released in May 2016
%----------------------------------------------------------------------

function G_n=G_Ja(x)


uAS=x(1);vAS=x(2);wAS=x(3);phi_n=x(4);theta_n=x(5);%psi=x(9);


% if num == 6
G_n=[ -1  0  0     0            wAS                   -vAS;...
      0  -1  0    -wAS         0                       uAS;...
      0  0  -1    vAS        -uAS                     0  ;...
      0  0  0     -1   -sin(phi_n)*tan(theta_n)    -cos(phi_n)*tan(theta_n);...
      0  0  0      0         -cos(phi_n)               sin(phi_n) ;...
      0  0  0      0   -sin(phi_n)/cos(theta_n)    -cos(phi_n)/cos(theta_n);
  ];