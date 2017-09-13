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
for i=1:length(ts)
    plot([xs(1,i) xs(1,i)+L*cos(xs(3,i))],[xs(2,i) xs(2,i)+L*sin(xs(3,i))],'r-')
    plot(xs(1,i),xs(2,i) ,'ko')

    hold on
end


%% 2. Simulation map generation - revolving door 

% obstacle rotation matrix 
obs_test=obstacle2(SE2([2 1],pi/4),[1 0.5]);
obs_test.plot
obs_test.isobs([3 2;2.5 1.5]')







