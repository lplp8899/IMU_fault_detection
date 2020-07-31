%%----------------------------------------------------------------------
% This is the code for the following journal paper:
%
%      P. Lu, L. Van Eykeren, E. van Kampen, C. C. de Visser, Q. P. Chu
%      Aircraft Inertial Measurement Unit Fault Identification with Application to Real Flight Data
%      Journal of Guidance, Control and Dynamics, 38(12):2467-2475
%      DOI: 10.2514/1.G001247
%
% and also can be adapted for the following journal paper:
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
%   Author: Peng Lu, Delft University of Technology
%
%   email:  P.Lu-1@tudelft.nl
%
%   released in May 2016
%----------------------------------------------------------------------

%% NOTE the sensor dynamics is included in the measurements
tic;

clear all
close all
% clc


%
disp('----------------------------------------')
disp('          You are running KM1')
disp('This one will not work when there are turbulence')
disp('----------------------------------------')
%

% sampling time, refer to 132, 140 in the sim_ac.m
delta_t = 0.01;

% load the data without turbulence
load data_sim_no_turbulence
% load the data with turbulence
load data_sim_turbulence.mat 



r2d = 180/pi;
% d2r = pi/180;


% true states
Vtas   =  Out_nonoise(:,1)';
alpha  =  Out_nonoise(:,2)';
beta   =  Out_nonoise(:,3)';

pb     =  Out_nonoise(:,4)';
qb     =  Out_nonoise(:,5)';
rb     =  Out_nonoise(:,6)';

phi    =  Out_nonoise(:,9)';
theta  =  Out_nonoise(:,8)';
psi    =  Out_nonoise(:,7)';

xe     =  Out_nonoise(:,10)';
ye     =  Out_nonoise(:,11)';
ze     =  Out_nonoise(:,12)'; % opposite sign of he
u_b    =  Out_nonoise(:,13)';
v_b    =  Out_nonoise(:,14)';
w_b    =  Out_nonoise(:,15)';

u_n    =  Out_nonoise(:,16)';
v_n    =  Out_nonoise(:,17)';
w_n    =  Out_nonoise(:,18)'; % the opposite sign of hdot


% with noise
A_x    =  squeeze(Ax_m)';
A_y    =  squeeze(Ay_m)';
A_z    =  squeeze(Az_m)';

% turbulence
ug_b     = Tur_b(:,1)';
vg_b     = Tur_b(:,2)';
wg_b     = Tur_b(:,3)';


% grond velocity in the body axis 
u_n_b = u_n.*cos(theta).*cos(psi)+v_n.*cos(theta).*sin(psi)-w_n.*sin(theta);
v_n_b = u_n.*(sin(theta).*cos(psi).*sin(phi)-sin(psi).*cos(phi))...
    +v_n.*(sin(theta).*sin(psi).*sin(phi)+cos(psi).*cos(phi))...
    +w_n.*cos(theta).*sin(phi);
w_n_b = u_n.*(sin(theta).*cos(psi).*cos(phi)+sin(psi).*sin(phi))...
    +v_n.*(sin(theta).*sin(psi).*cos(phi)-cos(psi).*sin(phi))...
    +w_n.*cos(theta).*cos(phi);



% lower the dimension
Out_noise=squeeze(Out_noise)';
% measurenments
p_m         =  Out_noise(:,4)';
q_m         =  Out_noise(:,5)';
r_m         =  Out_noise(:,6)';

Ax_m        =  squeeze(Ax_m)'; % due to the simulation
Ay_m        =  squeeze(Ay_m)';
Az_m        =  squeeze(Az_m)';

Vt_m        =  Out_noise(:,1)';
alpha_m     =  Out_noise(:,2)';
beta_m      =  Out_noise(:,3)';

phi_m       =  Out_noise(:,9)';
theta_m     =  Out_noise(:,8)';
psi_m       =  Out_noise(:,7)';
ze_m        =  Out_noise(:,12)'; % opposite sign of he_m
% ze          =  Out_noise(:,17)';
xe_m        =  Out_noise(:,10)';
ye_m        =  Out_noise(:,11)';


u_n_m       =  Out_noise(:,16)';
v_n_m       =  Out_noise(:,17)';
w_n_m       =  Out_noise(:,18)'; % Vz= -hedot, Vz and wn are in the same direction


% the input to the Kinematic model       
u  =  [Ax_m; Ay_m; Az_m; p_m; q_m; r_m];
[dim_u,gen] = size(u);
lameda = [ 0; 0; 0; 0; 0; 0 ]*ones(1,gen);

x_real = [ Vtas.*cos(alpha).*cos(beta); Vtas.*sin(beta); Vtas.*sin(alpha).*cos(beta); phi; theta; psi;];

% measurement of the kinmatic model
z_real = [ Vt_m;  alpha_m;  beta_m;  phi_m; theta_m;  psi_m;  ];




%-----------------    add the  actuator (IMU) faults start      ----------------
% total num of fault kinds
faults_in=zeros(6,gen);
%

fy=0.001;
faults_in(1,1001:3000)=faults_in(1,1001:3000)+1;
faults_in(2,1001:3000)=faults_in(2,1001:3000)+fy*(1:2000);
faults_in(3,1001:3000)=faults_in(3,1001:3000)+2*sin(0.005*pi*(1001:3000));
faults_in(4,1001:3000)=faults_in(4,1001:3000)+0.5/57.3;
faults_in(5,1001:3000)=faults_in(5,1001:3000)+1/57.3*sin(0.005*pi*(1:2000));
faults_in(6,1001:3000)=faults_in(6,1001:3000)+1/57.3;
% fy=0.001;
faults_in(1,4001:6000)=faults_in(1,4001:6000)-1*sin(0.005*pi*(1:2000));
faults_in(2,4001:6000)=faults_in(2,4001:6000)-fy*(1:2000);
faults_in(3,4001:6000)=faults_in(3,4001:6000)-2;
faults_in(4,4001:6000)=faults_in(4,4001:6000)-0.5/57.3;
faults_in(5,4001:6000)=faults_in(5,4001:6000)-0.5/57.3;
faults_in(6,4001:6000)=faults_in(6,4001:6000)-1/57.3*sin(0.005*pi*(1:2000));

% 
u(1,:)=u(1,:)+faults_in(1,:);
u(2,:)=u(2,:)+faults_in(2,:);
u(3,:)=u(3,:)+faults_in(3,:);
u(4,:)=u(4,:)+faults_in(4,:);
u(5,:)=u(5,:)+faults_in(5,:);
u(6,:)=u(6,:)+faults_in(6,:);


%-----------------      actuator (IMU) faults end        ----------------



%-------------------------------------------------------------------------

%------------------------------step 1    ------------------------------  
  

% initialisation
x_ob_0 = [0;  0;  0; 0;  0; 0; ];
P_x0 =  1e0*eye(6);


% acceleration of gravity
% g = y(:,28)';
g = 9.81*ones(1,gen);

nameda_factor = 1;

%-------------------------------------------------------------------------

%------------------------------ initialization of variables ------------------------------
[dim_sys,~] = size(x_ob_0);
[dim_out,~] = size(z_real);
dim_f_i=6;
dim_f_o=3;
dim_d=3;
%
x_ob = zeros(dim_sys,1);
z_ob = zeros(dim_out,1);
x_ob_mean = zeros(dim_sys,gen);
z_ob_mean = zeros(dim_out,gen);
inno = zeros(dim_out,gen);
norm_inno = zeros(1,gen);
error_x = zeros(6,gen);
norm_error_x = zeros(dim_sys,gen);
x_ob_filter = zeros(dim_sys,gen);
z_ob_filter = zeros(dim_out,gen);
P_ob_filter = zeros(dim_sys,dim_sys,gen);
residual = zeros(dim_out,gen);
norm_residual = zeros(1,gen);

f_ob=zeros(dim_f_o,gen);
Pf_ob=zeros(dim_f_o,dim_f_o,gen);
d_ob=zeros(dim_d,gen);
Pd_ob=zeros(dim_d,dim_d,gen);
error_f = zeros(6,gen);
norm_error_f = zeros(6,gen);


Qk = diag([1e-4,1e-4,1e-4,3e-8,3e-8,3e-8]);
Rk=diag([1e-2,3e-6,3e-6,3e-8,3e-8,3e-8]);

max_iter=500;% record the iteration no. of IEKF
epsu_crit=1e-8;

% OTSKF
dim_bias = 6;


% initialization of the bias filter

b_ob=zeros(dim_bias,gen);
Pb_ob=zeros(dim_bias,dim_bias,gen);
x_hat = zeros(dim_sys,1);
Px_hat = zeros(dim_sys,dim_sys,gen);


b_ob_0 = [ 1e-3; 1e-3; 1e-3; 1e-3; 1e-3; 1e-3]; % input faults

Pb0 = 1e0*eye(dim_bias);
Pxb0 = 1e-6*eye(dim_sys);
V0 = Pxb0/Pb0;
x_ob_0 = x_ob_0-V0*b_ob_0;
% bias_0 = b0; 
Px_0 = P_x0-V0*Pb0*V0';
Pb_0 = Pb0;
Q_b = diag([1e-4,1e-4,1e-4,3e-8,3e-8,3e-8]);

Q_xb = 0;


%----------------------- beginning of the IOTSEKF
%----------------------- start the state and fault estimation

for k=1:6000
% k=1;

    
    %------------------      bias-free filter       ------------------
    F=F_Ja(x_ob_0,u(:,k),g(:,k));
    
    % noise distribution matrix
    G_noise=G_Ja(x_ob_0);
 
    [Phi,Gamma]=c2d(F,G_noise,delta_t);

    % bias term in the bias-free filter
    % Coupling of the bias and bias-free filter
    % Refer to the paper for more details
    
    F_in = Gamma; % input faults
    
    U_bar = V0+F_in;
    
    Pb_k0 = Pb_0 + Q_b;
    
    U = U_bar + (Q_xb-U_bar*Q_b)*inv(Pb_k0);
    
    u_0 = (U_bar-U)*b_ob_0;

    Q_bar = Gamma*Qk*Gamma' - Q_xb*U_bar'-U*(Q_xb-U_bar*Q_b)';
    
    P=Phi*P_x0*Phi' + Q_bar;
    
    % predict
    x_ob=x_ob_0+model_fx(x_ob_0,u(:,k),g(:,k))*delta_t + u_0;
  
    H=H_Ja(x_ob);
    
    z_ob=model_output(x_ob);
    
    inno(:,k)=z_real(:,k)-z_ob;
    
    
%     %-----------------------------    IEKF  -------------------------------- 
    %-----------------------------    Iterated EKF  -------------------------------- 
    % Please refer to the conference version for why we do this iteration.
    % Doing this iteration, then filter is not sensitive to the initial
    % condition.
    eta_1=x_ob;
    num_iter=0;
    flag=1;
    
    while flag==1
%         % Jacobi matrix
        H=H_Ja(eta_1);
%         
        V=H*P*H'+Rk;
% 
        K=P*H'/V;
% 
        z_ob=model_output(eta_1);

        % residual
        inno(:,k)=z_real(:,k)-z_ob;
%         % norm of the residual
%         norm_inno(:,k)=norm(inno(:,k));
% 
        eta_2=x_ob+K*(inno(:,k)-H*(x_ob-eta_1));

        epsu(:,k)=norm(eta_2-eta_1)./norm(eta_1);
% 
        if (k<=10)&&(epsu(:,k)>epsu_crit) && (num_iter<max_iter)
%         if (k<=10) && (num_iter<max_iter)
            eta_1=eta_2;
            num_iter=num_iter+1;
            disp('------iteration------')
        else 
            flag=0;
        end
    end
% 
%     %predict    
    x_ob_iekf=eta_2;
    
    P_ob_iekf=(eye(dim_sys)-K*H)*P*(eye(dim_sys)-K*H)'+K*Rk*K';
% %-----------------------------  end of  IEKF  -------------------------------- 
% 
    x_ob = x_ob_iekf;
    P = P_ob_iekf;
    
    L = K;



    x_ob_filter(:,k)= x_ob;
    
    P_ob_filter(:,:,k)= P;
    
     
    %------------------------- bias filter ------------------------
    S = H*U;
    
    Kb = Pb_k0*S'*inv(H*P*H'+Rk+S*Pb_k0*S');
    
    
    b_ob(:,k) = b_ob_0+Kb*(inno(:,k)-S*b_ob_0);
    
    Pb_ob(:,:,k) = (eye(dim_bias)-Kb*S)*Pb_k0;
    %------------------------- bias filter ------------------------
  
    
    %----------------       coupled equations     ----------------
    
    V = U - L*S;
    x_hat(:,k) = x_ob_filter(:,k)+V*b_ob(:,k);
    
    Px_hat(:,:,k) = P_ob_filter(:,:,k)+V*Pb_ob(:,:,k)*V';
    
    %----------------       coupled equations     ----------------
    
    
    % next generation
    x_ob_0=x_ob_filter(:,k);
    P_x0=P_ob_filter(:,:,k);
    

    %
    b_ob_0 = b_ob(:,k);
    Pb_0 = Pb_ob(:,:,k);
    V0 = V;



end




% draw the figures
% 
Time=delta_t*(1:k);

% x
figure;
subplot(321); hold on; plot(Time,x_real(1,1:k),'r--'); plot(Time,x_hat(1,1:k),'b'); ylabel('u_a (m/s)','fontsize',13);grid;
    
subplot(323); hold on; plot(Time,x_real(2,1:k),'r--'); plot(Time,x_hat(2,1:k),'b'); ylabel('v_a (m/s)','fontsize',13);grid;
    
subplot(325); hold on; plot(Time,x_real(3,1:k),'r--'); plot(Time,x_hat(3,1:k),'b'); ylabel('w_a (m/s)','fontsize',13);grid;
    
subplot(322); hold on; plot(Time,x_real(4,1:k),'r--'); plot(Time,x_hat(4,1:k),'b'); ylabel('\phi (rad)','fontsize',13);grid;
    
subplot(324); hold on; plot(Time,x_real(5,1:k),'r--'); plot(Time,x_hat(5,1:k),'b'); ylabel('\theta (rad)','fontsize',13);grid;
    
subplot(326); hold on; plot(Time,x_real(6,1:k),'r--'); plot(Time,x_hat(6,1:k),'b'); ylabel('\psi (rad)','fontsize',13);grid;
    
legend('true','estimation');



% fault estimation
figure;
subplot(311);hold on; plot(Time,faults_in(1,1:k),'r--');plot(Time,b_ob(1,1:k),'b');  ylabel('f_{Ax} (m/s^2)','fontsize',13);grid; 
    
subplot(312);hold on; plot(Time,faults_in(2,1:k),'r--');plot(Time,b_ob(2,1:k),'b');  ylabel('f_{Ay} (m/s^2)','fontsize',13);grid; 
    
subplot(313);hold on; plot(Time,faults_in(3,1:k),'r--');plot(Time,b_ob(3,1:k),'b');  ylabel('f_{Az} (m/s^2)','fontsize',13);grid; 
    
h2 = legend('true','estimation'); set(h2,'color','white','edgecolor','black');

h1=axes('position',[0.52 0.0001 0.0001 0.0001],'fontsize',13);axis off;
title('time (s)','fontsize',13)

figure;
subplot(311);hold on; plot(Time,faults_in(4,1:k),'r--');plot(Time,b_ob(4,1:k),'b'); ylabel('f_p (rad/s)','fontsize',13);grid; 
    
subplot(312);hold on; plot(Time,faults_in(5,1:k),'r--');plot(Time,b_ob(5,1:k),'b'); ylabel('f_q (rad/s)','fontsize',13);grid; 
    
subplot(313);hold on; plot(Time,faults_in(6,1:k),'r--');plot(Time,b_ob(6,1:k),'b'); ylabel('f_r (rad/s)','fontsize',13);grid; 
    
h2= legend('true','estimation'); set(h2,'color','white','edgecolor','black');
h1=axes('position',[0.52 0.0001 0.0001 0.0001],'fontsize',13);
title('time (s)','fontsize',13)


toc