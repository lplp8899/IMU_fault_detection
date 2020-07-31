%%----------------------------------------------------------------------
% This is the code for the following journal paper:
%
%      P. Lu, L. Van Eykeren, E. van Kampen, C. C. de Visser, Q. P. Chu
%      Aircraft Inertial Measurement Unit Fault Identification with Application to Real Flight Data
%      Journal of Guidance, Control and Dynamics, 38(12):2467-2475
%      DOI: 10.2514/1.G001247
%
% and can also be adapted for the following paper:
%
%      P. Lu, E. van Kampen, C. C. de Visser, Q. P. Chu
%      Nonlinear aircraft sensor fault reconstruction in the presence of disturbances validated by real flight data
%      Control Engineering Practice 49:112-128
%      DOI: 10.1016/j.conengprac.2016.01.012
%
%      If you use this code, please refer to our papers. 
%
%   Also refer to the following conference paper for more details:
%   P. Lu and  E. van Kampen.
%   Aircraft Inertial Measurement Unit Fault Identification with Application to Real Flight Data
%   AIAA Guidance, Navigation, and Control Conference, 2015

%   If you have more questions, please contact the author:
%
%   Author: Peng Lu, Delft University of Technology,
%
%   email:  P.Lu-1@tudelft.nl
%
%   released in May 2016
%----------------------------------------------------------------------




run the main program:

Run_IOTSEKF_KM1.m % this one uses the KM1, and only works when there is no turbulence



Run_IOTSEKF_KM2.m % this one uses the KM2, and still works when there is turbulence


each main program has its own subfunctions, refer to the code for more details.