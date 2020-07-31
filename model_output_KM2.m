%----------------------------------------------------------------------
%   Author: Peng Lu, Delft University of Technology 
%   email:  P.Lu-1@tudelft.nl
%   released in May 2016
%----------------------------------------------------------------------

function y_ob = model_output_KM2(x)


uGS_b = x(1,:);vGS_b = x(2,:);wGS_b = x(3,:);
phi = x(4,:);theta = x(5,:);psi = x(6,:);

% ground speed 
uGS_e = (uGS_b.*cos(theta)+(vGS_b.*sin(phi)+wGS_b.*cos(phi)).*sin(theta)).*cos(psi)...
    -(vGS_b.*cos(phi)-wGS_b.*sin(phi)).*sin(psi);
vGS_e = (uGS_b.*cos(theta)+(vGS_b.*sin(phi)+wGS_b.*cos(phi)).*sin(theta)).*sin(psi)...
    +(vGS_b.*cos(phi)-wGS_b.*sin(phi)).*cos(psi);
wGS_e = -uGS_b.*sin(theta)+(vGS_b.*sin(phi)+wGS_b.*cos(phi)).*cos(theta);

% measurement 
y_ob = [uGS_e;
    vGS_e;
    wGS_e;
     phi;
     theta;
     psi;];

       
       