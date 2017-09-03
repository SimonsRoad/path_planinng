function [xddot,wdot,Rdot]=dynamics(R,w,f)
% R=rotation matrix
% x=position 
M=f(2:end);
global m g J


xddot=-g*[0 0 1]'+R*[0 0 f(1)]'/m;
Rdot=skew(w)*R;
wdot=inv(J)*(M-skew(w)*(J*w));

end