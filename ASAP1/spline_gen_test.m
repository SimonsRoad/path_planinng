%% spline generation vs one chuck 
% spline order 
n=7;
waypoint={};
x0=[0 0];
xdot0=[0 0];
waypoint{1}=[1 2];
waypoint{2}=[2 0];
waypoint{3}=[3 2];
% obstacles 
obstacles={};
obstacles{1}=[0.25 1.5];
obstacles{2}=[1.3 2.5];

w_d=1e+10; % weight for passing through the waypoints 
w_o=1e+3; % for obstacle avoidance 
sp=spline_path(n,x0,xdot0,waypoint,w_d,w_o,obstacles);
sp.generate_spline(); 
% sp.generate_one_poly();
sp.path_plot();





