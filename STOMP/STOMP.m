%% sensor reading test

% real map contruction 
real_map=robotics.OccupancyGrid(10,10,20);

obs_x=[3 7]; dx=[1 1];
obs_y=[5 3]; dy=[3 1];
Nx=200; Ny=200;
Nobs=length(obs_x);

for I =1: Nobs

    [xs,ys]=meshgrid(linspace(obs_x(I)-dx(I),obs_x(I)+dx(I),Nx),linspace(obs_y(I)-dy(I),obs_y(I)+dy(I),Ny));
    obstacle=[reshape(xs,[],1) reshape(ys,[],1)];
    setOccupancy(real_map,obstacle,1)
end

real_map.show()

pose = [1,1,0];
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


%% cost metric 

xs=[1 3 6 9];
ys=[1 3 5 9];

hold on 

plot(xs,ys,'r-')
cost=0;
occval=[];
for i=1:length(xs)-1
    [endPts,midPts] = occu_map.raycast([xs(i) ys(i)],[xs(i+1) ys(i+1)]);
    occval=[occval ;occu_map.getOccupancy([midPts;endPts],'grid')];
end



%% STOMP 

% real map contruction 

obs_x=[3 7]; 
obs_y=[5 3]; 

global obs_list
obs_list=[obs_x;obs_y];
ref_p=[1 2]';

global safety_rad
safety_rad=2;

figure()

plot(obs_x,obs_y,'r*','MarkerSize',7)
hold on 


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

x0=[1 1]';
xN=[9 9]';

X=linspace(x0(1),xN(1),N); Y=linspace(x0(2),xN(2),N);

cost_X=sum(cost_obs([X;Y]))+0.5*X*R*X';
cost_Y=sum(cost_obs([X;Y]))+0.5*Y*R*Y';


plot(X,Y,'b-')
% k noisy trajectories generation
K=10;
Xs=repmat(X,K,1); eXs=zeros(K,N);
Ys=repmat(Y,K,1); eYs=zeros(K,N);

for k=1:K
    eXs(k,:)=mvnrnd(zeros(N,1),Rinv);
    eYs(k,:)=mvnrnd(zeros(N,1),Rinv);
    plot(X(1,:)+eXs(k,:),X(2,:)+eYs(k,:),'k')
end

Xs=Xs+eXs; Ys=Ys+eYs;

% cost computing (same for x traj and y traj)

cost_mat=zeros(K,N);
for k=1:K
    cost_mat(k,:)=cost_obs([Xs(k,:) ;Ys(k,:)]);
end

min_cost=min(cost_mat);
max_cost=max(cost_mat);

h=10;

% for n=1:N
%     S=(cost_mat(:,n)-min_cost(n))/(max_cost(n)-min_cost(n));
%     S(S~=S)=inf; % if nan
%     cost_mat(:,n)=exp(-h*S);
% end

for n=1:N
    cost_mat(:,n)=exp(-h*cost_mat(:,n));
end

cost_mat=cost_mat./sum(cost_mat);

dX_hat=zeros(1,N);
dY_hat=zeros(1,N);
for n=1:N
    dX_hat(n)=cost_mat(:,n)'*eXs(:,n);
    dY_hat(n)=cost_mat(:,n)'*eYs(:,n);
end

dX=M*dX_hat';
dY=M*dY_hat';

X(1,:)=X(1,:)+dX'; X(2,:)=X(2,:)+dY';


cost_X=sum(cost_obs([X;Y]))+0.5*X*R*X';
cost_Y=sum(cost_obs([X;Y]))+0.5*Y*R*Y';








