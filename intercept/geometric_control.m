%% geometric controller test 
% parameters
clear all
addpath('Geometry-Toolbox\')

data.params.mQ = 0.5 ;
data.params.J = diag([0.557, 0.557, 1.05]*1e-2);
data.params.g = 9.81 ;
data.params.e1 = [1;0;0] ;
data.params.e2 = [0;1;0] ;
data.params.e3 = [0;0;1] ;
px=[0 0 6 -4];
py=[0 0 6 -4];
pz=[0 0 6 -4];
data.params.p=[px;py;pz];


%% trajectory calc  
R=eye(3);
w0=[0 0 0]';
x0=[zeros(6,1); reshape(R,9,1); w0]; 

disp('Simulating...') ;
odeopts = odeset('RelTol', 1e-8, 'AbsTol', 1e-9) ;
% odeopts = [] ;
[t, x] = ode15s(@UAV_dynamics_geometric_control, [0 5], x0, odeopts, data) ;

%% desired traj 
xd=[];
for i=1:length(t)
    t_=t(i);
    traj= get_flats([px;py;pz],t_);
    xd=[xd ; traj.x'];
end

%% plotting 
figure(1)
subplot(3,2,1)
plot(t,x(:,1))
hold on 
plot(t,xd(:,1))
title('x')

subplot(3,2,3)
plot(t,x(:,2))
hold on 
plot(t,xd(:,2))
title('y')

subplot(3,2,5)
plot(t,x(:,3))
hold on 
plot(t,xd(:,3))
title('z')

subplot(3,2,[2 4 6])
plot3(x(:,1),x(:,2),x(:,3),'k--')
hold on 
for ind=[1 300 400 550 700]
    Rplot(x(ind,1:3),reshape(x(ind,7:15),3,3),20)
end








