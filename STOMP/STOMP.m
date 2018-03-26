%% sensor reading test

% real map contruction 
real_map=robotics.OccupancyGrid(10,10,20);

obs_x=[4 7]; dx=[0.3 0.5];
obs_y=[5 3]; dy=[1 0.5];
Nx=200; Ny=200;
Nobs=length(obs_x);

for I =1: Nobs

    [xs,ys]=meshgrid(linspace(obs_x(I)-dx(I),obs_x(I)+dx(I),Nx),linspace(obs_y(I)-dy(I),obs_y(I)+dy(I),Ny));
    obstacle=[reshape(xs,[],1) reshape(ys,[],1)];
    setOccupancy(real_map,obstacle,1)
end

figure
real_map.show()


x0=[2 4]';

pose = [x0' 0];
maxrange = 10;
Nray=500;
angles = linspace(-pi/2+pi/4, pi/2+pi/4, Nray);
ranges=maxrange*ones(1,Nray);

for i=1:Nray
intPnt=real_map.rayIntersection(pose,angles(i),maxrange);
    if ~isnan(intPnt(1)) % if hits any obstacle
        ranges(i)=norm(pose(1:2)-intPnt);
    end    
end

% occumap contruction 
occu_map=robotics.OccupancyGrid(10,10,20);
occu_map.insertRay(pose,ranges,angles,maxrange)
occu_map.insertRay(pose,ranges,angles,maxrange)
mat=occu_map.occupancyMatrix;
occu_map.show()
hold on

%% STOMP 

N=50; % trajectory step number 
dt=0.1;
A=diag(-2*ones(1,N))+diag(ones(1,N-1),1)+diag(ones(1,N-1),-1);
A=A/(dt^2);
R=A'*A;
Rinv=inv(A'*A);
% scailing matrix M  
M=zeros(N);
for i=1:N
    M(:,i)=Rinv(:,i)/max(Rinv(:,i))/N;
end

% initial trajectory 

xN=[7 7]';

X=linspace(x0(1),xN(1),N); Y=linspace(x0(2),xN(2),N);

plot(X,Y,'b-')
% 
% cost_X=sum(cost_obs(X,Y))+0.5*X*R*X'
% cost_Y=sum(cost_obs(X,Y))+0.5*Y*R*Y'

cost=cost_occupancy(X,Y,occu_map)

%% 
% k noisy trajectories generation
plot(X,Y,'b-')

K=10;
Xs=repmat(X,K,1); eXs=zeros(K,N);
Ys=repmat(Y,K,1); eYs=zeros(K,N);

for k=1:K
    
    eXs(k,:)=mvnrnd(zeros(N,1),Rinv);
    eYs(k,:)=mvnrnd(zeros(N,1),Rinv);
%     plot(X+eXs(k,:),Y+eYs(k,:),'k')
end

Xs=Xs+eXs; Ys=Ys+eYs;

% cost computing (same for x traj and y traj)

cost_mat=zeros(K,N);
for k=1:K
    cost_mat(k,:)=cost_occupancy(Xs(k,:),Ys(k,:),occu_map);
end

min_cost=min(cost_mat);
max_cost=max(cost_mat);

h=10;

for n=1:N
    S=(cost_mat(:,n)-min_cost(n))/(max_cost(n)-min_cost(n));
    S(S~=S)=inf; % if nan
    cost_mat(:,n)=exp(-h*S);
end

% for n=1:N
%     cost_mat(:,n)=exp(-h*cost_mat(:,n));
% end

cost_mat=cost_mat./sum(cost_mat);

dX_hat=zeros(1,N);
dY_hat=zeros(1,N);
for n=1:N
    dX_hat(n)=cost_mat(:,n)'*eXs(:,n);
    dY_hat(n)=cost_mat(:,n)'*eYs(:,n);
end

dX=M*dX_hat';
dY=M*dY_hat';

X=X+dX'; Y=Y+dY';


cost=cost_occupancy(X,Y,occu_map)



%% move n waypoint 

N_move=10;

N_current=1;

for n=1:N_move
    pose=[X(N_current+n) Y(N_current+n) 0];
    
    angles = linspace(-pi/2+pi/4, pi/2+pi/4, Nray);
    ranges=maxrange*ones(1,Nray);
    
    for i=1:Nray
        intPnt=real_map.rayIntersection(pose,angles(i),maxrange);
        if ~isnan(intPnt(1)) % if hits any obstacle
            ranges(i)=norm(pose(1:2)-intPnt);
        end    
    end

    occu_map.insertRay(pose,ranges,angles,maxrange)
    mat=occu_map.occupancyMatrix;
    occu_map.show()
    hold on 
    plot(X,Y,'b-')
    plot(pose(1),pose(2),'r*')
    plot(xN(1),xN(2),'c*')
    pause(1)
   
end

%% path replanning 

% k noisy trajectories generation
plot(X,Y,'b-')


X=X(N_current+N_move:N); 
Y=Y(N_current+N_move:N);

N=N-N_move;
K=10;

A=diag(-2*ones(1,N))+diag(ones(1,N-1),1)+diag(ones(1,N-1),-1);
A=A/(dt^2);
R=A'*A;
Rinv=inv(A'*A);

M=zeros(N);
for i=1:N
    M(:,i)=Rinv(:,i)/max(Rinv(:,i))/N;
end


%%

Xs=repmat(X,K,1); eXs=zeros(K,N);
Ys=repmat(Y,K,1); eYs=zeros(K,N);

for k=1:K
    
    eXs(k,:)=mvnrnd(zeros(N,1),Rinv);
    eYs(k,:)=mvnrnd(zeros(N,1),Rinv);
%     plot(X+eXs(k,:),Y+eYs(k,:),'k')
end

Xs=Xs+eXs; Ys=Ys+eYs;

% cost computing (same for x traj and y traj)

cost_mat=zeros(K,N);
for k=1:K
    cost_mat(k,:)=cost_occupancy(Xs(k,:),Ys(k,:),occu_map);
end

min_cost=min(cost_mat);
max_cost=max(cost_mat);

h=10;

for n=1:N
    S=(cost_mat(:,n)-min_cost(n))/(max_cost(n)-min_cost(n));
    S(S~=S)=inf; % if nan
    cost_mat(:,n)=exp(-h*S);
end

% for n=1:N
%     cost_mat(:,n)=exp(-h*cost_mat(:,n));
% end

cost_mat=cost_mat./sum(cost_mat);

dX_hat=zeros(1,N);
dY_hat=zeros(1,N);
for n=1:N
    dX_hat(n)=cost_mat(:,n)'*eXs(:,n);
    dY_hat(n)=cost_mat(:,n)'*eYs(:,n);
end

dX=M*dX_hat';
dY=M*dY_hat';

X=X+dX'; Y=Y+dY';

plot(X,Y,'b-')


cost=cost_occupancy(X,Y,occu_map)



