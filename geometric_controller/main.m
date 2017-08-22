m=4; g=9.8;



dt=0.01;


R0=eye(3);
x=zeros(3,1); xdot=zeros(3,1);
w=[0 0 0]';

totstep=5/dt;

R=R0; 

xs=zeros(3,totstep);
Rs=zeros(3,3,totstep);
for i=1:totstep
    
    
    [xddot,wdot,Rdot]=dynamics(R,w,ones(4,1)*m*g/4);
    
    xdot=xdot+xddot*dt;
    x=x+xdot*dt;
    
    w=w+wdot*dt;
    R=R+Rdot*dt;
    
    xs(:,i)=x;
    Rs(:,:,i)=R;
   
end