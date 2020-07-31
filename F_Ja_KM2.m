%----------------------------------------------------------------------
%   Author: Peng Lu, Delft University of Technology 
%   email:  P.Lu-1@tudelft.nl
%   released in May 2016
%----------------------------------------------------------------------

function F=F_Ja_KM2(x,uk,gk)


phi_n=x(4);theta_n=x(5);


F(1,1)=0;
F(1,2)=(uk(6));
F(1,3)=-(uk(5));
F(1,4)=0;
F(1,5)=-gk*cos(theta_n);
F(1,6)=0;


F(2,1)=-(uk(6));
F(2,2)=0;
F(2,3)=(uk(4));
F(2,4)=gk*cos(theta_n)*cos(phi_n);
F(2,5)=-gk*sin(phi_n)*sin(theta_n);
F(2,6)=0;


F(3,1)=(uk(5));
F(3,2)=-(uk(4));
F(3,3)=0;
F(3,4)=-gk*sin(phi_n)*cos(theta_n);
F(3,5)=-gk*cos(phi_n)*sin(theta_n);
F(3,6)=0;


F(4,1:3)=[0,0,0];
F(4,4)=(uk(5))*cos(phi_n)*tan(theta_n)-(uk(6))*sin(phi_n)*tan(theta_n);
F(4,5)=(uk(5))*sin(phi_n)/(cos(theta_n)^2)+(uk(6))*cos(phi_n)/(cos(theta_n)^2);
F(4,6)=0;


F(5,1:3)=[0,0,0];
F(5,4)=-(uk(5))*sin(phi_n)-(uk(6))*cos(phi_n);
F(5,5:6)=[0,0];


F(6,1:3)=[0,0,0];
F(6,4)=(uk(5))*cos(phi_n)/cos(theta_n)-(uk(6))*sin(phi_n)/cos(theta_n);
F(6,5)=(uk(5))*sin(phi_n)*sin(theta_n)/(cos(theta_n)^2)+(uk(6))*cos(phi_n)*sin(theta_n)/(cos(theta_n)^2);
F(6,6)=0;










