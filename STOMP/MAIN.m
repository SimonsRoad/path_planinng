%% real map contruction 
global real_map 
real_map=robotics.OccupancyGrid(10,10,20);

obs_x=[3 7 8]; dx=[0.3 0.5 2];
obs_y=[5 9 6]; dy=[2 1 0.5];
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
x0=[1 1.2]';

pose = [x0' pi/4.5];
maxrange = 5;
Nray=100;

angle_min=-pi/8;
angle_max=pi/8;
global occu_map 
occu_map=robotics.OccupancyGrid(10,10,20);
% ray insertion using real_map and occu_map

rayinsertion(pose,angle_min,angle_max,Nray,maxrange)
mat=occu_map.occupancyMatrix;

occu_map.show()
hold on

%% initial  planning 
N_init=50; % initial time step 
dt=0.2; 
K=20; % noisy trajectories 
max_iter=10; 
verbose=true;
tol=0.2;
x0=[1 1]'; xN=[9 9]';

% plot(x0(1),x0(2),'c*')
% hold on 
% plot(xN(1),xN(2),'r*')
X0=linspace(x0(1),xN(1),N_init); Y0=linspace(x0(2),xN(2),N_init);
% plot(X0,Y0,'b-')
X0s=X0; Y0s=Y0;

N_guess=5;

% mutiple initial guess for global search 
%%
xc=(x0+xN)/2;
v=(xN-x0); v=[v(2) -v(1)]/norm(v);
pseudo_t=[0 0.5 1]; % this is for polyfit dummy input variable 

d=[1/4 1/2 -1/4 -1/2];

for i=1:length(d)
    xm=xc+v'*d(i)*norm(xN-x0);
    
    px=polyfit(pseudo_t,[x0(1) xm(1) xN(1)],2);
    py=polyfit(pseudo_t,[x0(2) xm(2) xN(2)],2);
    X0=polyval(px,linspace(0,1,N_init));
    Y0=polyval(py,linspace(0,1,N_init));
    X0s=[X0s; X0]; Y0s=[Y0s ;Y0];
end

%%

% occu_map.show()
plot(x0(1),x0(2),'c*')
plot(xN(1),xN(2),'r*')
hold on 
Xs=[]; Ys=[]; costs=[];
for i=1:N_guess
    fprintf('current guess: %d ----------------------\n',i);
    [X_perturbed,Y_perturbed,X_history,Ys_history,cost]=STOMP_fun(X0s(i,:),Y0s(i,:),K,dt,100,0.1,verbose);
    Xs=[Xs;X_perturbed]; Ys=[Ys;Y_perturbed]; costs=[costs cost];
    plot(X_perturbed,Y_perturbed,'r-')
end

%% move 

[~,min_idx]=min(costs);
X=Xs(min_idx,:); Y=Ys(min_idx,:);
figure
%%

n_move=10;
n_current=1;

%%

for i=1:n_move
    pose=[X(n_current+i) Y(n_current+i) atan2(Y(n_current+i)-Y(n_current+i-1),X(n_current+i),X(n_current+i-1))];
    % occupancy map update 
    rayinsertion(pose,angle_min,angle_max,Nray,maxrange)
    % re - evaluation
    if cost>1000
        disp('obstacle encountered')
    end
    
    cost=cost_occupancy(X(n_current+i:end),Y(n_current+i:end),occu_map);

    occu_map.show()
    hold on
    % plot path...
    plot(X,Y,'b-')
    plot(pose(1),pose(2),'c*')
    plot(xN(1),xN(2),'r*')
    pause(1e-1)
end

n_current=n_current+n_move;


%% replanning 
X0=X(n_current:end); Y0=Y(n_current:end);
[X,Y,X_history,Ys_history]=STOMP_fun(X0,Y0,K,dt,100,tol,verbose);

% plot path...
plot(X,Y,'b-')
plot(X(1),Y(1),'c*')
plot(xN(1),xN(2),'r*')


