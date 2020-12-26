function f = Give_friction(t,rhoR_dot,rhoL_dot,tR_dot,tL_dot,R,m)
%GIVE_FRICTION Summary of this function goes here
%   Detailed explanation goes here
mewR=0.8;
mewL=0.8;

N=m*9.81;
%Right wheel
if(abs(rhoR_dot)>abs(R*tR_dot)) %Case of braking
    if(abs(rhoR_dot)<10e-7)
        sr=0;
        flong_1=0;
    else
        sr=((abs(R*tR_dot)-abs(rhoR_dot))/(rhoR_dot));
        flong_1=Friction(mewR,sr,N);
    end
    
elseif(abs(rhoR_dot)<abs((R*tR_dot))) % Case of accelerating
    if(abs(R*tR_dot)<10e-7)
        flong_1=0;
        sr=0;
    else
        sr=((abs(R*tR_dot)-abs(rhoR_dot))/(R*tR_dot));
        flong_1=Friction(mewR,sr,N);
    end
else
     flong_1=0;
     sr=0;
end

%Left wheel
if(abs(rhoL_dot)>abs(R*tL_dot)) %Case of braking
    if(abs(rhoL_dot)<10e-7)
        flong_2=0;
        sl=0;
    else
        sl=((abs(R*tL_dot)-abs(rhoL_dot))/(rhoL_dot));
        flong_2=Friction(mewL,sl,N);
    end
elseif(abs(rhoL_dot)<abs((R*tL_dot))) % Case of accelerating
    if(abs(R*tL_dot)<10e-7)
        flong_2=0;
        sl=0;
    else
        sl=((abs(R*tL_dot)-abs(rhoL_dot))/(R*tL_dot));
        flong_2=Friction(mewL,sl,N);
    end
else
     flong_2=0;
     sl=0;
end
f=[flong_1;flong_2];

end

