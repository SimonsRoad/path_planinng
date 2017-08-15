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


%% planning - RRT* / only xyz
g=PGraph(3);
g.set_gamma(1);
ndim=3; ranges=[0 10; 0 10; 0 10];
obs{1}=[2 3;2 8;2 8]; obs{2}=[6 8;4 6;2 8];
prob=problem(ndim,ranges,obs); 
N=200;
root=[0.1 0.1 0.1]';
goal=[9 5.5 5.5]';
g.add_node(root);

prob.mapplot
hold on

isreached=0;
reach_tol=1e-2;
while g.n<5000
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
plot3(root(1),root(2),root(3),'r*')
plot3(goal(1),goal(2),goal(3),'r*')

p=g.Astar(1,g.closest(goal));
g.highlight_path(p)
%% N points generation 
N=3;
rot_mat=eul2r(pi/6,pi/6,pi/4); t=[1 1 1]'; 
T=SE3(rot_mat,t);

pnts=Npoint_gen(T,2,N);
figure()
plot3(pnts(1,:),pnts(2,:),pnts(3,:),'ko')





%% Plannin RRT* in xyz roll pitch yaw (see data1-3) ==> simplest case 
%% sampling
ndim=3; ranges=[0 20; 0 20; 0 20 ; 0 pi; 0 pi ;0 pi];
obs{1}=[2 3;2 8;2 8]; obs{2}=[6 8;4 6;2 8];
N=3;
prob=problem(ndim,ranges,obs,N); 
prob.mapplot;
hold on
% axis(reshape((ranges(1:3,:)*1.5).',1,[]))
q_rand=sample(prob);
q_next=q_rand+[-4 0 0 0.1 0.1 0.1]';
draw_drone(q_rand,1.5,0.5,2)
draw_drone(q_next,1.5,0.5,2)

%% planning - RRT* / xyz_rpy
g=PGraph(6,'distance','rpy','dweight',[2 1]);
g.set_gamma(1);
ndim=3; ranges=[0 20; 0 20; 0 20 ; 0 pi; 0 pi ;0 pi];
obs{1}=[5 6;8 14;2 8]; obs{2}=[13 17;8 11;2 8];
prob=problem(ndim,ranges,obs,3); 
N=200;
root=[0.1 0.1 0.1 0 0 0]';
goal=[16 16 8 0 0 0]';
g.add_node(root);

prob.mapplot
hold on

plot3(root(1),root(2),root(3),'r*')
plot3(goal(1),goal(2),goal(3),'r*')

%%

x_rand=sample(prob);
[v_nearest]=g.closest(x_rand);
% plot3(x_rand(1),x_rand(2),x_rand(3),'bo')
x_nearest=g.vertexlist(:,v_nearest);

x_new=steer(x_nearest,x_rand);

plot3(x_new(1),x_new(2),x_new(3),'b*')
v_new=g.add_node(x_new);
g.add_edge(v_nearest,v_new)

g.plot

%%
g=PGraph(6,'distance','rpy','dweight',[2 1]);
g.set_gamma(3);
ndim=3; ranges=[0 20; 0 20; 0 20 ; 0 pi; 0 pi ;0 pi];
obs{1}=[5 6;8 14;2 8]; obs{2}=[13 17;8 11;2 8];

prob=problem(ndim,ranges,obs,3); 
root=[0.1 0.1 0.1 0 0 0]';
goal=[16 16 8 0 0 0]';
g.add_node(root);

prob.mapplot
hold on

plot3(root(1),root(2),root(3),'r*')
plot3(goal(1),goal(2),goal(3),'r*')


%%

isreached=0;
reach_tol=1e-2;
while g.n<4000
    x_rand=sample(prob);
%     plot3(x_rand(1),x_rand(2),x_rand(3),'b*')
    [v_nearest]=g.closest(x_rand);
    x_nearest=g.vertexlist(:,v_nearest);
    x_new=steer(x_nearest,x_rand);
%     plot3(x_new(1),x_new(2),x_new(3),'bo')
    % obstacle free?
    if ~prob.isobs2(x_new,x_nearest)
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
            
            if ~prob.isobs2(x_near,x_new) && (c<c_min)
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
            if ~prob.isobs2(x_near,x_new) && (c<c_near)
                
                %finding x_parent 
                v_parent_cand=g.neighbours(v_near);
                v_parent=g.neighbours_in(v_near);
                
                for vv=v_parent_cand
                    if length(g.Astar(1,v_parent))>length(g.Astar(1,vv))
                            v_parent=vv;
                    end
                end
                
                
                g.delete_edge(intersect(g.edges(v_parent),g.edges(v_near)));
                g.add_edge(v_new,v_near);
                
            end
            
        
        end
        
    end 
end
%% post processing 
% g.plot

prob.mapplot
hold on

plot3(root(1),root(2),root(3),'r*')
plot3(goal(1),goal(2),goal(3),'r*')


p=g.Astar(1,g.closest(goal));

for v=p
    draw_drone(g.vertexlist(:,v),1.5,0.5,2)
    pause(1)
end

%% plotting 
g.highlight_path(p)







