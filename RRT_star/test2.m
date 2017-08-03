g=PGraph(2);
g.set_gamma(1);
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
reach_tol=1e-1;
while ~isreached
    x_rand=sample(prob);
    plot(x_rand(1),x_rand(2),'r*')
    [v_nearest]=g.closest(x_rand);
    x_nearest=g.vertexlist(:,v_nearest);
    x_new=steer(x_nearest,x_rand);
    % obstacle free?
    if ~prob.isobs(x_new,x_nearest)
        v_new=g.add_node(x_new);
        V_near=g.near(v_new);
        if isempty(V_near)
            break
        end
        % pick the node for minimum cost
        x_min=x_nearest;
        [~,c]=g.Astar(1,v_nearest);
        c_min=c+g.distance(v_nearest,v_new);
        
        for i=1:length(V_near)
            v_near=V_near(i);
            x_near=g.vertexlist(:,v_near); 
            [~,c]=g.Astar(1,v_near);
            c=c+g.distance(v_near,v_new);
            
            if ~prob.isobs(x_near,x_new) && (c<c_min)
                c_min=c; x_min=x_near;
            end
        end % for near
        v_min=g.add_node(x_min);
        g.add_edge(v_new,v_min);
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
p=g.Astar(1,g.n);
g.highlight_path(p)


