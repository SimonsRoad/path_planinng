%% real map contruction 
global real_map 
real_map=robotics.OccupancyGrid(10,10,20);

obs_x=[3 7]; dx=[0.3 0.5];
obs_y=[5 9]; dy=[2 1];
Nx=200; Ny=200;
Nobs=length(obs_x);

for I =1: Nobs

    [xs,ys]=meshgrid(linspace(obs_x(I)-dx(I),obs_x(I)+dx(I),Nx),linspace(obs_y(I)-dy(I),obs_y(I)+dy(I),Ny));
    obstacle=[reshape(xs,[],1) reshape(ys,[],1)];
    setOccupancy(real_map,obstacle,1)
end

figure
real_map.show()

%% occu_map 
x0=[1 1]';

pose = [x0' 0];
maxrange = 10;
Nray=500;

angle_min=-pi/2+pi/4;
angle_max=pi/2+pi/4;
global occu_map 
occu_map=robotics.OccupancyGrid(10,10,20);

% ray insertion using real_map and occu_map

rayinsertion(pose,angle_min,angle_max,Nray,maxrange)

occu_map.show()
hold on

%% initial  planning 
N_init=50; % initial time step 
dt=0.1; 
K=10; % noisy trajectories 
max_iter=10; 
verbose=true;
tol=3;
x0=[1 1]'; xN=[9 9]';
X0=linspace(x0(1),xN(1),N); Y0=linspace(x0(2),xN(2),N);

[X,Y,Xs,Ys]=STOMP_fun(X0,Y0,K,dt,max_iter,tol,verbose);


plot(X,Y,'b-')







