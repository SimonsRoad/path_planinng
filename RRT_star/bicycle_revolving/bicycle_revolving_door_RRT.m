% bicyle model main file

%% 1. Bicycle dynamics test 

% Specification 

dt=0.1; % this is RK numerical integration time step
tf=5; x0=zeros(3,1); u=[1 pi/6];
ts=0; x=x0; xs=[x]; 
L=1; % Currently this is not synchornized in dynamics.  


% Integration 
for t=0:dt:tf
    x_next=RK(x,u,dt,@bicycle_dynamics);
    xs=[xs x_next];
    ts=[ts t];
    x=x_next;
end


% Drawing 
figure()
for i=1:length(xs)
    plot([xs(1,i) xs(1,i)+L*cos(xs(3,i))],[xs(2,i) xs(2,i)+L*sin(xs(3,i))],'r-')
    plot(xs(1,i),xs(2,i) ,'ko')

    hold on
end


%% 2. Simulation map generation - revolving door 

% obstacle rotation matrix 
obs_test=obstacle2(SE2([2 1],pi/4),[1 0.5]);
obs_test.plot
obs_test.isobs([3 2;2.5 1.5]')


%% 3. Bicycle RRT*
% (1) Steering test
%  state = (x,y,theta,t) input=(linear_vel, steering angle)

s1=[0 0 0 0]; s2=[ 3 3 pi/4 2];
d=norm(s1(1:2)-s2(1:2)); delt=s2(4)-s1(4);

vs=d/delt*[0.5 1 1.5]; steer_angs=linspace(-pi/3,pi/3,8);
vmin=0; steer_angs_min=0; cost_min=100;

for v=vs
    for steer_ang=steer_angs
        x=s1(1:3)'; u=[v steer_ang]';
        xs=[]; ts=[];
        for t=0:dt:delt/2
            x_next=RK(x,u,dt,@bicycle_dynamics);
            xs=[xs x_next];
            x=x_next;
        end
        cost=norm(x-s2(1:3));
        if cost<cost_min
           cost_min=cost;
           xs_min=xs;
        end
    end
end

%% (2) Plain simulation - without any obstacle 
nstate=4; % s=[x y theta t] 
distance_measure='bicycle';
g=PGraph
        
        
        

       






