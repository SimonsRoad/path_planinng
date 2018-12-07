%% Phase 5 : Combination for divided regions 

% intersection region of the two polyhydra corresponding to each target
FOV = 160 * pi/180;
A_div = {};
b_div = {};
c_div = {}; % center of each convex polyhedra
v_div = {}; % convex vertex 

vis_cost_set = {};
Nk = 0; % number of valid regions 
ratio = 1; % importance ratio of vis2 to vis1
for h = 1:H    
    A_div{h} = {};
    b_div{h} = {};
    c_div{h} = {};
    v_div{h} = {};
    Nk = 0;
    
    for i = 1:length(visi_info_set{1}.cost_set{h})
        for j = 1:length(visi_info_set{2}.cost_set{h})
            
            % feasibility test for the intersection region of the two
            % polyhedron
                        
            Ai = visi_info_set{1}.A_set{h}{i};
            bi = visi_info_set{1}.b_set{h}{i};
            vis_cost1 = visi_info_set{1}.cost_set{h}(i);
            
            Aj = visi_info_set{2}.A_set{h}{j};
            bj = visi_info_set{2}.b_set{h}{j};
            vis_cost2 = visi_info_set{2}.cost_set{h}(j);
                              
            A_intsec = [Ai ; Aj];
            b_intsec = [bi ; bj];
            
%             % please wait 
%             verr1=con2vert([Ai; A_bound],[bi ; b_bound]);
%             verr2=con2vert([Aj; A_bound],[bj ; b_bound]);
            
                        
            A_bound = [1 0 0; -1 0 0; 0 1 0; 0 -1 0;0 0 1; 0 0 -1];
            b_bound = [xu ; -xl ; yu ; -yl ; zu ; -zl];
            
            [~,~,flag]=linprog([],[Ai ; Aj ],[bi ; bj] ,[],[],[xl yl zl],[xu yu zu]);
            
            
            if (flag ~= -2) % feasibility test pass
               % let's keep this region for now 
               % let's investigate the FOV constraint 
%                  vertices = con2vert([A_bound;A_intsec ],[;b_bound ;b_intsec]); % the vertices of this region   
               
                
                % In case of 3D, the intersection region of two polyhydron
                % can be 2D, which can paralyze the code 
%                 vertices = con2vert([A_bound;A_intsec ],[b_bound ;b_intsec]); % the vertices of this region   
                

               try  
                    vertices = con2vert([A_bound;A_intsec ],[;b_bound ;b_intsec]); % the vertices of this region   
               catch
                    vertices = [];
               end
               
               
               if ~isempty(vertices)
               
               vertices = vertices + 0.1 * (mean(vertices) - vertices);
               
               % we reject the segment if any of them is included in the blind region    
               height = 1;
%                if (sum(is_in_blind3([targets_xs(1,h) targets_ys(1,h) height],[targets_xs(2,h) targets_ys(2,h) height],FOV,vertices',0)) == 0)               
                   
                   Nk = Nk + 1;                     
                   A_div{h}{Nk} = A_intsec;
                   b_div{h}{Nk} = b_intsec; 
                   v_div{h}{Nk} =  vertices;
                   c_div{h}{Nk} = mean(vertices); % center of each segment                
                   
                   vis_cost_set{h}(Nk) =  vis_cost1 + ratio * vis_cost2; % let's assign the visibility cost to here    
%                end
               end
            end % feasibility test pass
                        
        end        
    end
end