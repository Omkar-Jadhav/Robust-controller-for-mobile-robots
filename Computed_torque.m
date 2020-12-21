function [Sdot] = Computed_torque(t,S,vr,wr,L,Kp,Kd,Iwy,R,...
                                    m,I,mc,b)
%COMPUTED_TORQUE Summary of this function goes here
%   Detailed explanation goes here

phiR=S(1);
phiL=S(2);
phiR_dot=S(3);
phiL_dot=S(4);
phiR_des=S(5);
phiL_des=S(6);

phiR_des_d= vr+wr*L;
phiL_des_d= vr-wr*L;

e=[phiR_des-phiR;...
    phiL_des-phiL];

phi_dot=[phiR_dot;phiL_dot]

V=Kp*e-Kd*phi_dot;

w=(phiR_dot-phiL_dot)/(2*L);

M=[Iwy+(R^2/(4*L^2))*(m*L^2+I),(R^2/(4*L^2))*(m*L^2-I);...
(R^2/(4*L^2))*(m*L^2-I),Iwy+(R^2/(4*L^2))*(m*L^2+I)];

C=[((R^2/(2*L)))*mc*b*w*phiL_dot;...
    -((R^2/(2*L)))*mc*b*w*phiR_dot];

u= (M*V+C);

phi_ddot=inv(M)*(u-C);

phiR_ddot=phi_ddot(1);
phiL_ddot=phi_ddot(2);

Sdot=[phiR_dot ;phiL_dot ;phiR_ddot ;phiL_ddot ;phiR_des_d ;phiL_des_d];
end


