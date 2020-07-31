%----------------------------------------------------------------------
%   Author: Peng Lu, Delft University of Technology 
%   email:  P.Lu-1@tudelft.nl
%   released in May 2016
%----------------------------------------------------------------------

function x_ob=model_fx_KM2(x,u,g,delta_t)
 

uGS_b=x(1,:);vGS_b=x(2,:);wGS_b=x(3,:);
phi=x(4,:);theta=x(5,:);psi=x(6,:);


Axm=u(1,:);Aym=u(2,:);Azm=u(3,:);
pm=u(4,:);qm=u(5,:);rm=u(6,:); 

x_ob=[   (Axm)-g.*sin(theta)+(rm).*vGS_b-(qm).*wGS_b;
     (Aym)+g.*cos(theta).*sin(phi)+(pm).*wGS_b-(rm).*uGS_b;
     (Azm)+g.*cos(theta).*cos(phi)+(qm).*uGS_b-(pm).*vGS_b;
     (pm)+(qm).*sin(phi).*tan(theta)+(rm).*cos(phi).*tan(theta);
     (qm).*cos(phi)-(rm).*sin(phi);
     (qm).*sin(phi)./cos(theta)+(rm).*cos(phi)./cos(theta);];
