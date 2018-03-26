function [x_new,u]=bicycle_steering(x_near,x_rand)
% This fucntion is for bicycle RRT 
% steering function 
% Issue: delt/2
dt=0.1; % integration 
s1=x_near; s2=x_rand;
d=norm(s1(1:2)-s2(1:2)); delt=s2(4)-s1(4); steer_time=delt/2;

vs=d/delt*[0.5 1 1.5]; steer_ang=linspace(-pi/3,pi/3,8);
vmin=0; steer_ang_min=0; cost_min=100;

for v=vs
    for steer_ang=steer_ang
        x=s1(1:3)'; u=[v steer_ang]';
        xs=[]; ts=[];
        for t=0:dt:steer_time
            x_next=RK(x,u,dt,@bicycle_dynamics);
            xs=[xs x_next];
            x=x_next;
        end
        cost=norm(x-s2(1:3));
        if cost<cost_min
           cost_min=cost;
           xs_min=xs; vmin=v; steer_ang_min=steer_ang;
        end
    end
end

x_new=[x ; (s1(4)+steer_time)]; u=[vmin steer_ang_min];
end
       


