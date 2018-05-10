%% test file for spline generation 
% the spline passed through given waypoints, avoding obstacles 
n=7; % spline order 
waypoint={};
x0=[0 0];
xdot0=[0 0];
waypoint{1}=[1 2];
waypoint{2}=[2 1];
waypoint{3}=[3 1];

% obstacle 
T1=SE2;   T2=SE2;
T1.t=[1.5 1.5]'; T2.t=[0.5 1]';
obs1_scale=[0.1 0.2];  obs2_scale=[0.2 0.2];
obs1=obstacle2(T1,obs1_scale); obs2=obstacle2(T2,obs2_scale);

% manager 
PM=path_manager({obs1,obs2});

PM.traj_gen(x0,xdot0,waypoint);

%% plot path 
hold on 
PM.mapplot()
hold on
PM.path_plot()
for i=1:length(waypoint)
    plot(waypoint{i}(1),waypoint{i}(2),'r*')
end







