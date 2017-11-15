function cost_vec=cost_obs(X,Y)

% x is state vector x=[xs;ys]
% obstacle_list=[obs_x; obs_y]
global safety_rad;
global obs_list

n=size(X,2); % time step per a trajectory
cost_vec=zeros(1,n);

for i = 1: n
    d_closest=min(distance([X(i) ;Y(i)],obs_list));
    cost_vec(i)=max(safety_rad-d_closest,0);
end