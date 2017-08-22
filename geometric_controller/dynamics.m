function [xddot,wdot,Rdot]=dynamics(R,w,f)
% R=rotation matrix
% x=position 
% f= thrust vector = [ f1 f2 f3 f4 ]

d=0.15; c=8e-4;

ft=sum(f);
M=[0 d 0 -d ; -d 0 d 0; c -c c -c]*f;
  
m=4; J=diag([0.0820,0.0845,0.1377]);
g=9.8;


xddot=-g*[0 0 1]'+R*[0 0 ft]'/m;
Rdot=skew(w)*R;
wdot=inv(J)*(M-skew(w)*(J*w));
end