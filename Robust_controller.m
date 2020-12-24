function [Sdot del_v1 del_v2]= Robust_controller(t,S,vr,wr,L,Kp,Kd,...
                R,mc,m,I,b,Iwy)
%ROBUST_CONTROLLER Summary of this function goes here
%   Detailed explanation goes here

rhoR=S(1);
rhoL=S(2);
rhoR_dot=S(3);
rhoL_dot=S(4);
rhoR_des=S(5);
rhoL_des=S(6);

phiR_des_d= (vr+wr*L);
phiL_des_d= (vr-wr*L);

% phiR_des_dd=vr;
% phiL_des_dd=vr;
% 
% phi_des_dd=[phiR_des_d;...
%             phiL_des_d];
        
e=[rhoR_des-rhoR;...
    rhoL_des-rhoL];

edot=[phiR_des_d-rhoR_dot;...
        phiL_des_d-rhoL_dot];


E=[e;edot];
phi_dot=[rhoR_dot;rhoL_dot];

V=Kp*e-Kd*phi_dot;

w=(rhoR_dot-rhoL_dot)/(2*L);

C=R*[((1/(2*L)))*mc*b*w*rhoL_dot;...
    -((1/(2*L)))*mc*b*w*rhoR_dot];

M=R*[2*Iwy/(R^2)+(1/(4*L^2))*(m*L^2+I),(1/(4*L^2))*(m*L^2-I);...
        (1/(4*L^2))*(m*L^2-I),2*Iwy/(R^2)+(1/(4*L^2))*(m*L^2+I)]; 

C_cap=0.5*C;

Mcap=R*[Iwy/(R^2)+(1/(4*L^2))*(m*L^2+I),(1/(4*L^2))*(m*L^2-I);...
        (1/(4*L^2))*(m*L^2-I),Iwy/(R^2)+(1/(4*L^2))*(m*L^2+I)]; 

K=[Kp Kd];              %Gains  
del_v=Del_v(M,Mcap,V,Iwy,R,m,L,I,K,C_cap,E);
del_v1=del_v(1);
del_v2=del_v(2);

u= (M*V+C);

phi_ddot=inv(M)*(u-C);

phiR_ddot=phi_ddot(1);
phiL_ddot=phi_ddot(2);

Sdot=[rhoR_dot ;rhoL_dot ;phiR_ddot ;phiL_ddot ;phiR_des_d ;phiL_des_d];
end

