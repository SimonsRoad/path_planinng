function [xl,vl,Rl,wl]=x2s(x)
% this function is parsing x to physical variables
xl=x(1:3); %2
vl=x(4:6); %2
Rl=reshape(x(7:15),3,3);  %4
wl=x(16:18);    %1 
end