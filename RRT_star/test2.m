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

%% planning - RRT* / xyz_rpy / Vijar scenario
g=PGraph(6,'distance','rpy','dweight',[1 1]);
g.set_gamma(3);


% simple scenario
% ndim=3; ranges=[0 20; 0 20; 0 20 ; 0 pi; 0 pi ;0 pi];
% obs{1}=[5 6;8 14;2 8]; obs{2}=[13 17;8 11;2 8];
% prob=problem(ndim,ranges,obs,3); 
% N=200;
% root=[0.1 0.1 0.1 0 0 0]';
% goal=[16 16 8 0 0 0]';
% g.add_node(root);

% agile scenario
ndim=3;
obs{1}=[0 2;3 15;-5 5];
obs{2}=[0 2;-15 -3;-5 5];
obs{3}=[0 2 ;-3 3;-5 -0.5];
obs{4}=[0 2;-3 3;0.5 5];

% for i=1:length(obs)
%     obs{i}=rotx(pi/6)*obs{i};
% end



ranges=[-10 10;-15 15 ; -5 5 ;0 pi/2; 0 pi/2 ;0 pi/2];
prob=problem(ndim,ranges,obs,3);



prob.mapplot
hold on
root=[-7 10 0 pi/6 0 0]';


g.add_node(root);
% g.add_node(zeros(1,6));
% g.add_edge(1,2)

draw_drone(root,1.5,0.5,2)
goal=[7 -10 0 0 0 0]';
draw_drone(goal,1.5,0.5,2)


plot3(root(1),root(2),root(3),'go','MarkerSize',5)
plot3(goal(1),goal(2),goal(3),'ro','MarkerSize',5)
xlabel('x')
ylabel('y')
zlabel('z')
grid on

draw_drone(zeros(1,6),1.5,0.5,2)





%%

isreached=0;
reach_tol=1e-2;
while g.n<3000
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



p=g.Astar(1,g.closest(goal));

for v=p

    
    
    prob.mapplot
    hold on

    plot3(root(1),root(2),root(3),'r*')
    plot3(goal(1),goal(2),goal(3),'r*')

    draw_drone(g.vertexlist(:,v),1.5,0.5,2)
    pause(1)
    
    
end

%% plotting 
g.highlight_path(p)


