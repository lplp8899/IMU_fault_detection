%----------------------------------------------------------------------
%   Author: Peng Lu, Delft University of Technology 
%   email:  P.Lu-1@tudelft.nl
%   released in May 2016
%----------------------------------------------------------------------

function x_ob=model_fx(x,u,g)


[num,den]=size(x);

uAS=x(1,:);vAS=x(2,:);wAS=x(3,:);
phi=x(4,:);theta=x(5,:);psi=x(6,:);

if num == 6
la_x=0;la_y=0;la_z=0;
la_p=0;la_q=0;la_r=0;
end

Axm=u(1,:);Aym=u(2,:);Azm=u(3,:);
pm=u(4,:);qm=u(5,:);rm=u(6,:); 


x_ob=[   (Axm-la_x)-g.*sin(theta)+(rm-la_r).*vAS-(qm-la_q).*wAS;
         (Aym-la_y)+g.*cos(theta).*sin(phi)+(pm-la_p).*wAS-(rm-la_r).*uAS;
         (Azm-la_z)+g.*cos(theta).*cos(phi)+(qm-la_q).*uAS-(pm-la_p).*vAS;
         (pm-la_p)+(qm-la_q).*sin(phi).*tan(theta)+(rm-la_r).*cos(phi).*tan(theta);
         (qm-la_q).*cos(phi)-(rm-la_r).*sin(phi);
         (qm-la_q).*sin(phi)./cos(theta)+(rm-la_r).*cos(phi)./cos(theta);];

     
     