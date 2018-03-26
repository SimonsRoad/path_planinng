function x=s2x(xl,vl,Rl,wl)
% this function is parsing x to physical variables
x=zeros(18,1);
x(1:3)=xl; %2
x(4:6)=vl; %2
x(7:15)=reshape(Rl,9,1);
x(16:18)=wl;    %1 
end