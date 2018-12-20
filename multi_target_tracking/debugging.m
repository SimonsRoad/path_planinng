%% Phase 5 : Combination for divided regions 

% intersection region of the two polyhydra corresponding to each target
FOV = 120 * pi/180;
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
    
    % we reject the blind region 
    rel_dist = norm([target1_xs(h) target1_ys(h)] - [target2_xs(h) target2_ys(h)]); % distance between the targets 
    blind_height = rel_dist/2/tan(FOV/2);
    
    
    
    for i = 1:length(visi_info_set{1}.cost_set_flat{h})
        for j = 1:length(visi_info_set{2}.cost_set_flat{h})
            
            % feasibility test for the intersection region of the two
            % polyhedron
                        
            Ai = visi_info_set{1}.A_set_flat{h}{i};
            bi = visi_info_set{1}.b_set_flat{h}{i};
            vis_cost1 = visi_info_set{1}.cost_set_flat{h}{i};
            
            Aj = visi_info_set{2}.A_set_flat{h}{j};
            bj = visi_info_set{2}.b_set_flat{h}{j};
            vis_cost2 = visi_info_set{2}.cost_set_flat{h}{j};
                              
            A_intsec = [Ai ; Aj];
            b_intsec = [bi ; bj];
            
%             % please wait 
%             verr1=con2vert([Ai; A_bound],[bi ; b_bound]);
%             verr2=con2vert([Aj; A_bound],[bj ; b_bound]);
            
                        
            A_bound = [1 0 0; -1 0 0; 0 1 0; 0 -1 0;0 0 1; 0 0 -1];
            b_bound = [xu ; -xl ; yu ; -yl ; zu ; -(max(target1_zs(h),target2_zs(h))+blind_height)];
            
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
                    warning('no vertex could be found')
                    vertices = [];
               end
               
               
               if ~isempty(vertices)
               
                   
               vertices = vertices + 0.1 * (mean(vertices) - vertices);
                              
               % we reject the segment the volume is too small                    
               shp = alphaShape(vertices(:,1),vertices(:,2),vertices(:,3),20);
               vol = shp.volume;
               if vol >= 4
                  Nk = Nk + 1;                     
                   A_div{h}{Nk} = [A_intsec; A_bound];
                   b_div{h}{Nk} = [b_intsec; b_bound]; 
                   v_div{h}{Nk} =  vertices;
                   c_div{h}{Nk} = mean(vertices); % center of each segment                
                   
                   vis_cost_set{h}(Nk) =  vis_cost1 + ratio * vis_cost2; % let's assign the visibility cost to here    
               end 
               
               end
            end % feasibility test pass
                        
        end        
    end
end