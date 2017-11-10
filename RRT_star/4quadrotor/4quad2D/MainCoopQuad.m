%% plot test 

% state
xl=zeros(3,1);
vl=zeros(3,1);
Rl=rotz(pi/6);
wl=zeros(3,1);
x0=s2x(xl,vl,Rl,wl);
% input
u0=[1 1 0 -1 1 0 -1 -1 0 1 -1 0   2 1 3 1]';

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

[t, x] = ode45(@CoopQuad2Dynmaics, tspan, x0, odeopts,data) ;




x1=x(end,:)';
[xl,vl,Rl,wl]=x2s(x1);
PlotState(x(end,:)',u0,data);


%% Steering test
kR=1; kx=1; kv=0.5; kw=0.4;

% q input selections
NQOption=5;
dtheta=pi/2/(NQOption+1);
qset0=[];
%qset0 is selection of q 
for i=1:NQOption
    qset0=[qset0 [cos(dtheta*(i)) sin(dtheta*(i)) 0]'];
end
% qset is each selection for 4 corners 
for i=1:4
   qset{i}= rotz(pi/2*(i-1))*qset0;
end
% T set is also 
Tset=0.2:0.2:1;

% let's pick the pair (qi,Ti) whose cost is the least 
% first, we need to construct the input pairs 
for i=1:4
    uset{i}.q=[];
    uset{i}.T=[];
    for j=1:NQOption
        for k=1:length(Tset)
            uset{i}.q=[uset{i}.q qset{i}(:,j)];
            uset{i}.T=[uset{i}.T,Tset(k)];
        end
    end
end

Cmin=1e+5; % intialized cost 
umin=zeros(16,1);
count=0;
tot=(NQOption*length(Tset))^4;
for i1=1:NQOption*length(Tset)
    for i2=1:NQOption*length(Tset)
        for i3=1:NQOption*length(Tset)
            for i4=1:NQOption*length(Tset)
                u=zeros(16,1);
                u(1:3)=uset{1}.q(:,i1); 
                u(4:6)=uset{2}.q(:,i2);
                u(7:9)=uset{3}.q(:,i3);
                u(10:12)=uset{4}.q(:,i4);
                u(13:end)=[uset{1}.T(i1) uset{2}.T(i2) uset{3}.T(i3) uset{4}.T(i4)];
                data.u=u;
                [t, x] = ode45(@CoopQuad2Dynmaics, tspan, x0, odeopts,data) ;
                count=count+1;
                fprintf('current: %d / %d\n',count,tot)
                xend=x(end,:)';
                C=cost_metric(x0,xend); 
                if C<Cmin
                    umin=u;  xmin=xend;
                end
            end
        end
    end
end

    
























