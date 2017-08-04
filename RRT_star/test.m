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


%% planning - RRT

g=PGraph(2);
g.set_gamma(1);
ndim=2; ranges=[0 10; 0 10];
obs{1}=[2 3;2 8]; obs{2}=[6 8;4 6];
prob=problem(ndim,ranges,obs); 
N=200;
root=[0.1 0.1]';
goal=[9 5.5]';
g.add_node(root);


isreached=0;
reach_tol=1e-1;
while ~isreached
    x_rand=sample(prob);
    [v_nearest]=g.closest(x_rand);
    x_nearest=g.vertexlist(:,v_nearest);
%     plot(x_rand(1),x_rand(2),'r*')
    x_new=steer(x_nearest,x_rand);
    
    if ~prob.isobs(x_new,x_nearest)
        v_new=g.add_node(x_new);
        g.add_edge(v_nearest,v_new)
        isreached=norm(x_new-goal)<reach_tol;
    end 
end

% g.plot
prob.mapplot
hold on
plot(root(1),root(2),'r*')
plot(goal(1),goal(2),'r*')
p=g.Astar(1,g.n);
g.highlight_path(p)


%% planning - RRT*
g=PGraph(2);
g.set_gamma(0.5);
ndim=2; ranges=[0 10; 0 10];
obs{1}=[2 3;2 8]; obs{2}=[6 8;4 6];
prob=problem(ndim,ranges,obs); 
N=200;
root=[0.1 0.1]';
goal=[9 5.5]';
g.add_node(root);

prob.mapplot
hold on

isreached=0;
reach_tol=1e-2;
while g.n<20000
    x_rand=sample(prob);
%     plot(x_rand(1),x_rand(2),'r*')
    [v_nearest]=g.closest(x_rand);
    x_nearest=g.vertexlist(:,v_nearest);
    x_new=steer(x_nearest,x_rand);
    % obstacle free?
    if ~prob.isobs(x_new,x_nearest)
        v_new=g.add_node(x_new);
        V_near=g.near(v_new);
        % pick the node for minimum cost
        x_min=x_nearest;  v_min=v_nearest;
        [~,c]=g.Astar(1,v_nearest);
        c_min=c+g.distance(v_nearest,v_new);
        
        for i=1:length(V_near)
            v_near=V_near(i);
            x_near=g.vertexlist(:,v_near); 
            [~,c]=g.Astar(1,v_near);
            c=c+g.distance(v_near,v_new);
            
            if ~prob.isobs(x_near,x_new) && (c<c_min)
                c_min=c; x_min=x_near; v_min=v_near;
            end
        end % for near
        
        g.add_edge(v_min,v_new);
        isreached=norm(x_new-goal)<reach_tol;
        
        % rewiring 
        for i=1:length(V_near)
            v_near=V_near(i);
            x_near=g.vertexlist(:,v_near); 
            
            [~,c_new]=g.Astar(1,v_new);
            c=c_new+g.distance(v_near,v_new);
            [~,c_near]=g.Astar(1,v_near);
            if ~prob.isobs(x_near,x_new) && (c<c_near)
                v_parent=g.neighbours_in(v_near);
                g.delete_edge(intersect(g.edges(v_parent),g.edges(v_near)));
                g.add_edge(v_near,v_new);
            end
            
        
        end
        
    end 
end

% g.plot
prob.mapplot
hold on
plot(root(1),root(2),'r*')
plot(goal(1),goal(2),'r*')

p=g.Astar(1,g.closest(goal));
g.highlight_path(p)




