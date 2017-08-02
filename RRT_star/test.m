%% Linearization test
% perturbation test
global l
l=1;
% % equilibium point
% x0=[20 20 20 0 0 0 zeros(1,6)]';
% u0=[9.81 0 0 0]';

% random point
x0=5*rand(12,1);
u0=6*rand(4,1);


x0dot=dynamics(x0,u0);

xq=x0+0.5*rand(12,1);
uq=u0+0.5*rand(4,1);

xqdot=dynamics(xq,uq);

% 
% [A,B]=linearize(x0,u0);

[A,B,xqdot_app]=linearize(x0,u0,xq,uq);

%% LQR simulation 

% linearized system
x_init=0.25*rand(6,1);
global Q R
Q=10*diag([2 2 2 0.1 0.1 2 2 2 2 0.1 0.1 2]); R=0.01*eye(4);
[K,S]=lqr(A,B,Q,R,0);
sys=ss(A-B*K,zeros(12,4),eye(6,12),0);
t0=0; tf=6; dt=0.01;
t=t0:dt:tf;
nb=length(t);
lsim(sys,ones(length(t),4),t,[x_init;zeros(6,1)])




% actual system 
x=[x_init;zeros(6,1)]+x0;
xlist=[]; tlist=[];
tlist=0;
xlist=x(1:6);
for t=t0:dt:tf-4
xdot=dynamics(x,-K*(x-x0)+u0);
[~,~,xdot_app]=linearize(x0,u0,x,-K*(x-x0)+u0);
delta=norm(xdot-xdot_app);
dx=xdot*dt;
x=x+dx;
xlist=[xlist x(1:6)];
tlist=[tlist t];
end

% figure()
% for i=1:6
%     subplot(6,1,i)
%     plot(tlist,xlist(i,:))
%     hold on
%     plot(tlist,)
% end


%% planning

% N=200;
% ndim=6+6;
% ranges=[-10 20; -10 20; -10 20; 0 2*pi ;0 2*pi;0 2*pi;...
%     -10 20; -10 20; -10 20; 0 2*pi ;0 2*pi;0 2*pi];

% making map 
load map1
sim_map=map;
g=PGraph(2);
ndim=2; ranges=[0 10; 0 10];

prob=problem(ndim,ranges,sim_map); 
sample(prob)

