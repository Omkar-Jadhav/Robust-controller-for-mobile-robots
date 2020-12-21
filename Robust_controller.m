function [Sdot del_v1 del_v2]= Robust_controller(t,S,vr,wr,L,Kp,Kd,...
                R,mc,m,I,b,Iwy)
%ROBUST_CONTROLLER Summary of this function goes here
%   Detailed explanation goes here

phiR=S(1);
phiL=S(2);
phiR_dot=S(3);
phiL_dot=S(4);
phiR_des=S(5);
phiL_des=S(6);

phiR_des_d= (vr+wr*L)/R;
phiL_des_d= (vr-wr*L)/R;

% phiR_des_dd=vr;
% phiL_des_dd=vr;
% 
% phi_des_dd=[phiR_des_d;...
%             phiL_des_d];
        
e=[phiR_des-phiR;...
    phiL_des-phiL];

edot=[phiR_des_d-phiR_dot;...
        phiL_des_d-phiL_dot];


E=[e;edot];
phi_dot=[phiR_dot;phiL_dot];

V=Kp*e-Kd*phi_dot;

w=(phiR_dot-phiL_dot)/(2*L);

C=[((R^2/(2*L)))*mc*b*w*phiL_dot;...
    -((R^2/(2*L)))*mc*b*w*phiR_dot];

M=[2*Iwy+(R^2/(4*L^2))*(m*L^2+I),(R^2/(4*L^2))*(m*L^2-I);...
        (R^2/(4*L^2))*(m*L^2-I),2*Iwy+(R^2/(4*L^2))*(m*L^2+I)]; 

C_cap=0.5*C;

Mcap=[Iwy+(R^2/(4*L^2))*(m*L^2+I),(R^2/(4*L^2))*(m*L^2-I);...
        (R^2/(4*L^2))*(m*L^2-I),Iwy+(R^2/(4*L^2))*(m*L^2+I)]; 

K=[Kp Kd];              %Gains  
del_v=Del_v(M,Mcap,V,Iwy,R,m,L,I,K,C_cap,E);
del_v1=del_v(1);
del_v2=del_v(2);

u= (M*V+C);

phi_ddot=inv(M)*(u-C);

phiR_ddot=phi_ddot(1);
phiL_ddot=phi_ddot(2);

Sdot=[phiR_dot ;phiL_dot ;phiR_ddot ;phiL_ddot ;phiR_des_d ;phiL_des_d];
end

