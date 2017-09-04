
global dt nb J
global m g
m=4; g=9.8; dt=0.1;
J=diag([0.0820,0.0845,0.1377]);

Trajectory_generation
%initial condition 
R0=[-1 0 0;0 -1 0;0 0 1];
r=rd(:,1)+[-0.5 0 0]'; rdot=zeros(3,1); 
w=[0 0 0]'; % angular velocity 

R=R0; 
rs=zeros(3,nb); % position history
Rs=zeros(3,3,nb); % rotation matrix history 
Rds=zeros(3,3,nb);

draw_drone2(R,r,1,0.5,2)

f=zeros(4,1);
Kp=eye(3)/2; Kv=eye(3)/2; KR=eye(3)/10; Kw=eye(3)/10;
axis([-10 10 -10 10 -3 3])
hold on
%time integration 

items = get(gca, 'Children');


for i=1:nb-2    
    
    F_des=(-Kp*(r-rd(:,i))-Kv*(rdot-rd_dot(:,i))+m*g*[0 0 1]'+m*rd_ddot(:,i));
    u1=F_des'*R(:,3);
    
    zb_des=F_des/norm(F_des);
    yb_des=cross(zb_des,[cos(yawd(i)) sin(yawd(i)) 0]'); yb_des=yb_des/norm(yb_des);
    xb_des=cross(yb_des,zb_des);
    
    
    Rd=[xb_des yb_des zb_des];
    
    trplot([Rd r; 0 0 0 1]);
    
    Rds(:,:,i)=Rd;
    
    
    if i==1
        wd=zeros(3,1);
    else
        wd=vex(Rd'*(Rd-Rds(:,:,i-1))/dt);
    end
        
    eR=0.5*vex(Rd'*R-R'*Rd);
    ew=R'*w-R'*Rd*wd;
    
    u234=-KR*eR-Kw*ew;
    
    f=[u1 ;u234];
    
    [rddot,wdot,Rdot]=dynamics(R,w,f);
    
    rdot=rdot+rddot*dt;
    r=r+rdot*dt;
    
    w=w+wdot*dt;
    R=R+Rdot*dt;
    
    rs(:,i)=r;
    Rs(:,:,i)=R;
    

    axis([-10 10 -10 10 -3 3])
    
    draw_drone2(R,r,1,0.5,2)    
    
    pause(1)
end

%plotting





