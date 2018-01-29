%% geometric controller test 
t0=0; tf=10;
N_sim=100;
ts=linspace(t0,tf,N_sim);
dt=ts(2)-ts(1);
xs=linspace(1,11,N_sim); ys=zeros(1,N_sim); zs=zeros(1,N_sim);
Xs=[xs; ys; zs];
Vs=[(xs(2)-xs(1))/dt*ones(1,N_sim); zeros(2,N_sim)];
As=zeros(3,N_sim);

for 