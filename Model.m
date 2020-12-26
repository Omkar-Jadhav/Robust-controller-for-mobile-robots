%% 
clc
clear all
close all
tic
%% Defining variables
syms rho1 rho2 t1 t2 eta1 tau1 tau2 omega V T_d T_t mr b vr omega_r R ...
        t1_ddot t2_ddot t1_dot t2_dot L Vdot omega_dot Tr Tl Meq Jeq
   

%% Parameters
mc = 2; % Mass of the Robot platform
mr=mc;
mw=0.05; % Mass of the wheel and motor
R = 0.0364; % Radius of the wheel
b= 0.05; % Distance of the robot from the center of drive axle
L = 0.164; % Distance between two wheels

Im=0.0050;
Iwy=0.0025;
Ic=0.732;
I=Ic+mc*b^2+2*mw*L^2+2*Im;   %MI of robot wrt centre point A.

m=mc+2*mw;

%% Solver
timespan=0:0.01:5;
IC=[0;0;0;0;0;0;0;0];

vr=1;
wr=1;

wn=3;
Kp=diag([wn^2,wn^2]);
Kd=diag([2*wn,2*wn]);

% [t S]=ode45(@(t,S) Computed_torque(t,S,vr,wr,L,Kp,Kd,Iwy,R,...
%                                     m,I,mc,b),timespan,IC);

[t S]=ode23(@(t,S)  Robust_controller(t,S,vr,wr,L,Kp,Kd,...
                R,mc,m,I,b,Iwy),timespan,IC);
            
[~,del_v1, del_v2, I_tilde Tr Tl]=cellfun(@(t,S) Robust_controller(t,S,vr,wr,L,Kp,Kd,...
                R,mc,m,I,b,Iwy),num2cell(t),...
                            num2cell(S,2),'uni',0);
                        
del_v1=cell2mat(del_v1);
del_v2=cell2mat(del_v2);
I_tilde=cell2mat(I_tilde);
Tr=cell2mat(Tr);
Tl=cell2mat(Tl);
%% Plots
rhoR_dot=S(:,3);
rhoL_dot=S(:,4);

v=(rhoR_dot+rhoL_dot)/2;
w=(rhoR_dot-rhoL_dot)/(2*L);

figure;
subplot(1,2,1)
plot(t,v,'Linewidth',2);
title('$V$','Interpreter','Latex')
xlim([0,10])
grid on
subplot(1,2,2)
plot(t,w,'Linewidth',2)
title('$\omega$','Interpreter','Latex')

xlim([0,10])
grid on

figure;
subplot(1,2,1)
plot(t,del_v1,'Linewidth',2)
title('$\Delta v_1$','Interpreter','Latex')
grid on
box on
subplot(1,2,2)
plot(t,del_v2,'Linewidth',2)
grid on 
box on
title('$\Delta v_2$','Interpreter','Latex')

figure;
plot(t,I_tilde,'Linewidth',2)
box on
grid on

figure;
subplot(1,2,1)  
plot(t,Tr,'Linewidth',2)
title('$\tau_R$','Interpreter','Latex')
box on
grid on
subplot(1,2,2)
plot(t,Tl,'Linewidth',2)
box on
title('$\tau_L$','Interpreter','Latex')
grid on
toc