%----------------------------------------------------------------------
%   Author: Peng Lu, Delft University of Technology 
%   email:  P.Lu-1@tudelft.nl
%   released in May 2016
%----------------------------------------------------------------------

function H=H_Ja(x)


uAS=x(1);vAS=x(2);wAS=x(3);
 

H(1,1)=uAS/sqrt(uAS^2+vAS^2+wAS^2);
H(1,2)=vAS/sqrt(uAS^2+vAS^2+wAS^2);
H(1,3)=wAS/sqrt(uAS^2+vAS^2+wAS^2);
H(1,4:6)=zeros(1,3);



H(2,1)=-wAS/(uAS^2+wAS^2);
H(2,2)=0;
H(2,3)=uAS/(uAS^2+wAS^2);
H(2,4:6)=zeros(1,3);

H(3,1)=-uAS*vAS/(1+vAS^2/(uAS^2+wAS^2))/(uAS^2+wAS^2)^(3/2);
H(3,2)=1/(1+vAS^2/(uAS^2+wAS^2))/sqrt(uAS^2+wAS^2);
H(3,3)=-wAS*vAS/(1+vAS^2/(uAS^2+wAS^2))/(uAS^2+wAS^2)^(3/2);
H(3,4:6)=zeros(1,3);


H(4:6,:)=[zeros(3,3),eye(3)];

     
         




