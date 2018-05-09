%% spline generation vs one chuck 
% spline order 
n=7;
waypoint={};
x0=[0 0];
xdot0=[0 0];
waypoint{1}=[1 2];
waypoint{2}=[2 1];
waypoint{3}=[3 2];

% obstacle 

T1=SE2;   T2=SE2;
T1.t=[1.5 1.5]'; T2.t=[0.5 1]';
obs1_scale=[0.1 0.2];  obs2_scale=[0.2 0.2];
obs1=obstacle2(T1,obs1_scale); obs2=obstacle2(T2,obs2_scale);

% manager 
PM=path_manager({obs1,obs2});
figure()
axis([-1 4 -1 4])
lower_left=[1 1]-0.2*[1 1];
upper_right=[2 2]+0.2*[1 1];
Nx=15; Ny=15;
x0=[1 2]; xf=[2 1];
N_pnts=3;
GPs=PM.guidance_waypoint(x0,xf,lower_left,upper_right,Nx,Ny,N_pnts);
PM.mapplot()
hold on
plot(GPs(:,1),GPs(:,2),'gs-')

%%

vset=(waypoint{1}-x0)';

for i=2:length(waypoint)
    vset=[vset ; waypoint{i}(1)-waypoint{i-1}(1) ; waypoint{i}(2)-waypoint{i-1}(2) ];
end

sum_cost=@(v) poly_traj_gen(v,n,x0,xdot0,waypoint,PM);

% two-stage optimization 

upper_limit=2;
lower_limit=-2;
len=length(vset);

options = optimoptions('fmincon','Algorithm','SQP');
sum_cost(vset);
[vset,final_jerk]=fmincon(sum_cost,vset,[],[],[],[],lower_limit*ones(len,1),upper_limit*ones(len,1),[],options);


%% plot 
hold on 

PM.path_plot()
for i=1:length(waypoint)
    plot(waypoint{i}(1),waypoint{i}(2),'r*')
end







