        for i=1:length(V_near)
            
            
            
            v_near=V_near(i);
            x_near=g.vertexlist(:,v_near); 
            
            [~,c_new]=g.Astar(1,v_new);
            c=c_new+g.distance(v_near,v_new);
            [~,c_near]=g.Astar(1,v_near);
                
                %finding x_parent 
                v_parent_cand=g.neighbours(v_near);
                v_parent=g.neighbours_in(v_near);
                
                for vv=v_parent_cand
                    if length(g.Astar(1,v_parent))>length(g.Astar(1,vv))
                            v_parent=vv;
                    end
                end
                
%                 
%                 g.delete_edge(intersect(g.edges(v_parent),g.edges(v_near)));
%                 g.add_edge(v_new,v_near);
%                 
            
            
        
        end