%% STOMP function 
% written by JBS 2017 / 11/ 16 
function [X,Y,Xs,Ys,cost_old]=STOMP_fun(X0,Y0,K,dt,max_iter,tol,verbose,varargin)

% X,Y: trajectory 
% Xs, Ys: trajectory history 
% tol : gradient difference between each iteration 

global occu_map

N=length(X0); % trajectory step number 
A=diag(-2*ones(1,N))+diag(ones(1,N-1),1)+diag(ones(1,N-1),-1);
A=A/(dt^2);
R=A'*A;
Rinv=inv(A'*A);
% scailing matrix M  
M=zeros(N);
for i=1:N
    M(:,i)=Rinv(:,i)/max(Rinv(:,i))/N;
end

%% trajectory history

Xs=X0;
Ys=Y0;

%% initial trajectory 
X=X0 ; Y=Y0;

cost_old=cost_occupancy(X,Y,occu_map,varargin);



iter_count=0;
cost_diff=1000;

if verbose
    fprintf('current iteration step: %d / cost : %f \n',iter_count,cost_old);
end



while ((iter_count<max_iter) && cost_diff>tol) 
%% k noisy trajectories generation

% for stuck situation in local minima 
if cost_old>1000
    var_correction=(cost_old/1000)^3;
else
    var_correction=0.8;
end

Xks=repmat(X,K,1); eXs=zeros(K,N);
Yks=repmat(Y,K,1); eYs=zeros(K,N);

for k=1:K
    eXs(k,:)=mvnrnd(zeros(N,1),var_correction*Rinv);
    eYs(k,:)=mvnrnd(zeros(N,1),var_correction*Rinv);
end

Xks=Xks+eXs; Yks=Yks+eYs;

%% cost computing (same for x traj and y traj)

cost_mat=zeros(K,N);
for k=1:K
    cost_mat(k,:)=cost_occupancy(Xks(k,:),Yks(k,:),occu_map,varargin);
end

min_cost=min(cost_mat);
max_cost=max(cost_mat);

h=5;

for n=1:N
    S=(cost_mat(:,n)-min_cost(n))/(max_cost(n)-min_cost(n));
    %S(S~=S)=inf; % if nan
    cost_mat(:,n)=exp(-h*S);
end

cost_mat=cost_mat./sum(cost_mat);

dX_hat=zeros(1,N);
dY_hat=zeros(1,N);
for n=1:N
    dX_hat(n)=cost_mat(:,n)'*eXs(:,n);
    dY_hat(n)=cost_mat(:,n)'*eYs(:,n);
end

%% computing gradient 

dX=M*dX_hat';
dY=M*dY_hat';

X=X+dX'; Y=Y+dY';
Xs=[Xs ; X]; Ys=[Ys ; Y];
cost_new=cost_occupancy(X,Y,occu_map,varargin);


cost_diff=abs(cost_new-cost_old);
cost_old=cost_new;

iter_count=iter_count+1;


if verbose
    fprintf('current iteration step: %d / cost : %f \n',iter_count,cost_old);
end


end




end