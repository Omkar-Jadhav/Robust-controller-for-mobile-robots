function [x_filtered, x_vec]= Use_filter(x_vec,x_update,windowSize)
%USE_FILTER Summary of this function goes here
%   Detailed explanation goes here
a = 1; 
b = (1/windowSize)*ones(1,windowSize);

x_vec=[x_vec,x_update]; % Updating I_tilde vec with new updated value
x_vec=x_vec(:,end-3:end); %Taking the latest 4 values only

y = filter(b,a,x_vec,[],2);

x_filtered=y(:,end); %Using the latest filtered value

end

