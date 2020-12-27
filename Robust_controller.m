function [Sdot del_v1 del_v2 I_tilde Tr Tl]= Robust_controller(t,S,vr,wr,L,Kp,Kd,...
                R,mc,m,I,b,Iwy)
%ROBUST_CONTROLLER Summary of this function goes here
%   Detailed explanation goes here
rhoR=S(1);
rhoL=S(2);
rhoR_dot=S(3);
rhoL_dot=S(4);
rhoR_des=S(5);
rhoL_des=S(6);
tR_dot=S(7);
tL_dot=S(8);

%% Desired wheel linear velocities
rhoR_des_d= (vr+wr*L);
rhoL_des_d= (vr-wr*L);
%% Creating global variables 
% This global variables will be used for calculation of I_tilde
global u rhoR_ddot tR_ddot tL_ddot I_tilde Tr Tl

if(t==0)
    Tr=0;
    Tl=0;
    I_tilde=Iwy;
    tR_ddot=0;
    tL_ddot=0;
    u=[0;0];
    rhoR_ddot=0;
end
%% Updating I_tilde
% I_tilde is the Iwy which accounts for the disturbance on the system.
f=Give_friction(t,rhoR_dot,rhoL_dot,tR_dot,tL_dot,R,m);   %Friction on the wheels
flong_1=f(1);
flong_2=f(2);

%%
Tr=u(1);    % Getting the torques on the wheel
Tl=u(2);    % Getting the torques on the wheel

tR_ddot=(Tr-flong_1*R)/Iwy;
tL_ddot=(Tl-flong_2*R)/Iwy;

if(abs(rhoR_ddot)<=0.1)         % To prevent overshooting of I_tilde when rho_ddot brcomes too small
    I_tilde=Iwy;
else
    I_tilde=Iwy*R*tR_ddot/rhoR_ddot;
end
%%
w=(rhoR_dot-rhoL_dot)/(2*L);    % Actual omega

C=R*[((R^2/(2*L)))*mc*b*w*rhoL_dot;...
    -((R^2/(2*L)))*mc*b*w*rhoR_dot];         % Actual C matrix

M=R*[I_tilde/(R^2)+(1/(4*L^2))*(m*L^2+I),(1/(4*L^2))*(m*L^2-I);...
        (1/(4*L^2))*(m*L^2-I),I_tilde/(R^2)+(1/(4*L^2))*(m*L^2+I)]; %Actual M matrix

C_cap=0.5*C; % Estimated C 

Mcap=R*[Iwy/(R^2)+(1/(4*L^2))*(m*L^2+I),(1/(4*L^2))*(m*L^2-I);...
        (1/(4*L^2))*(m*L^2-I),Iwy/(R^2)+(1/(4*L^2))*(m*L^2+I)];     % Estimated M matrix

%% Control

e=[rhoR_des-rhoR;...
    rhoL_des-rhoL]; % Error terms

edot=[-rhoR_dot;...
      -rhoL_dot];   % Error terms


E=[e;edot]; % Error terms

V=Kp*e+Kd*edot;      % Contoller input

K=[Kp Kd];              %Gains  
del_v=Del_v(M,Mcap,V,Iwy,R,m,L,I,K,C_cap,E); % Calculate delta vterm
del_v1=del_v(1);            %For plotting
del_v2=del_v(2);            %For plotting

u= (Mcap*(V+del_v)+C_cap);  % Final control

rho_ddot=inv(M)*(u-C);      % Closed loop equation

rhoR_ddot=rho_ddot(1);
rhoL_ddot=rho_ddot(2);
%% 
Sdot=[rhoR_dot ;rhoL_dot ;rhoR_ddot ;rhoL_ddot ;rhoR_des_d ;rhoL_des_d;tR_ddot;tL_ddot];
end