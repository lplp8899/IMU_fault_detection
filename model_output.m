%----------------------------------------------------------------------
%   Author: Peng Lu, Delft University of Technology 
%   email:  P.Lu-1@tudelft.nl
%   released in May 2016
%----------------------------------------------------------------------
function y_ob=model_output(x)

uAS=x(1,:);vAS=x(2,:);wAS=x(3,:);
phi=x(4,:);theta=x(5,:);psi=x(6,:);

% comes from the measurement of the sensors
y_ob=[   sqrt(uAS.^2+vAS.^2+wAS.^2);
         atan(wAS./uAS);
         atan(vAS./sqrt(uAS.^2+wAS.^2));    
         phi;
         theta;
         psi;];

     
     
     
       