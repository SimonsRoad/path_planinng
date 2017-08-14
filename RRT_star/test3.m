
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
                
%                 g.highlight_node(v_near,'NodeFaceColor','b')
%                 g.highlight_node(v_parent,'NodeFaceColor','g')
%                 g.highlight_node(v_new,'NodeFaceColor','r')
                
                
                del_edge=intersect(g.edges(v_parent),g.edges(v_near));
                g.delete_edge(del_edge);
                g.add_edge(v_new,v_near);
                
            end
            
        
        end
        
    end 
end