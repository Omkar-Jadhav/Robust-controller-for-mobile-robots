%% 
clc
clear all
close all
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
Meq=m+(2*Iwy/R^2);
Jeq=(I+2*Iwy*L^2/R^2);

%% Solver
timespan=0:0.01:10;
IC=[0;0;0;0;0;0];

vr=1;
wr=0;

wn=2;
Kp=diag([wn^2,wn^2]);
Kd=diag([2*wn,2*wn]);

% [t S]=ode45(@(t,S) Computed_torque(t,S,vr,wr,L,Kp,Kd,Iwy,R,...
%                                     m,I,mc,b),timespan,IC);

[t S]=ode23(@(t,S)  Robust_controller(t,S,vr,wr,L,Kp,Kd,...
                R,mc,m,I,b,Iwy),timespan,IC);
            
[~,del_v1, del_v2]=cellfun(@(t,S) Robust_controller(t,S,vr,wr,L,Kp,Kd,...
                R,mc,m,I,b,Iwy),num2cell(t),...
                            num2cell(S,2),'uni',0);
                        
del_v1=cell2mat(del_v1);
del_v2=cell2mat(del_v2);
%% Plots
phiR_dot=S(:,3);
phiL_dot=S(:,4);

v=(phiR_dot+phiL_dot)*R/2;
w=(phiR_dot-phiL_dot)*R/(2*L);

figure;
subplot(1,2,1)
plot(t,v,'Linewidth',2);
hold on
plot(t,t,'--','Linewidth',2)
hold off
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
title('$\delta v_1$','Interpreter','Latex')
grid on
box on
subplot(1,2,2)
plot(t,del_v2,'Linewidth',2)
grid on 
box on
title('$\delta v_2$','Interpreter','Latex')