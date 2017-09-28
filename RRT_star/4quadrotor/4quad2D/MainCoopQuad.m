%% plot test 

% state
xl=zeros(3,1);
vl=zeros(3,1);
Rl=rotz(pi/6);
wl=zeros(3,1);
x0=s2x(xl,vl,Rl,wl);
% input
u0=[1 1 0 -1 1 0 -1 -1 0 1 -1 0      2 1 3 1]';

data.ri=[2 1 0 ; -2 1 0; -2 -1 0; 2 -1 0]';
data.ml=1; 
data.Jl=diag([1/12*data.ml*((2*data.ri(2,1))^2 + (2*data.ri(3,1))^2) ...
    1/12*data.ml*((2*data.ri(1,1))^2 + (2*data.ri(3,1))^2) ...
    1/12*data.ml*((2*data.ri(1,1))^2 + (2*data.ri(2,1))^2)]);

data.u=u0;


figure
PlotState(x0,u0,data);
hold on

%% Numerical integration 

odeopts = odeset('RelTol', 1e-9, 'AbsTol', 1e-10) ;
% odeopts = [] ;
tspan=[0 1];

[t, x] = ode15s(@CoopQuad2Dynmaics, tspan, x0, odeopts,data) ;

[xl,vl,Rl,wl]=x2s(x(end,:)');

PlotState(x(end,:)',u0,data);


%% Steering test
% q input selections
NQOption=5;
dtheta=pi/2/(NQOption+1);
qset0=[];
for i=1:NQOption
    qset0=[qset0 [cos(dtheta*(i)) sin(dtheta*(i)) 0]'];
end

for i=1:4
   qset{i}= rotz(pi/2*(i-1))*qset0;
end

Tset=0.2:0.2:1;

% Á¿°°³×

















