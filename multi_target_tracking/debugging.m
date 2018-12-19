%% Phase2: Get score maps and inequality condition set of each target during a prediction horizon

% parameter for observation 
d_ref = 2;
N_azim = [40,40];
N_elev = [10, 10];

% parameter for sub-division             
N_rect = 10; r_max_stride = 10; c_max_stride = 10; stride_res = 1;                

% visi info 
visi_info_set = {}; % idx = 1 : target1 / idx =2 : target2

for n = 1:N_target 
    target_xs = targets_xs(n,:);
    target_ys = targets_ys(n,:);
    target_zs = targets_zs(n,:);
    visi_info = {}; % information on At_set,bt_set,visi_cost_set_t
    
    cost_set_t = {}; % 1th idx  : time / 2nd idx : just index in the time 
    A_set_t = {}; % set of affine region at the time t
    b_set_t = {};
    rect_set_t ={}; % four corner of of each rect 
    DT_t = {}; % distance field of each agent at time t 
    
    for t = 1:length(target_xs) % -1 is due to some mistake in set_target_tracker function 
         Nt = 0; % number of sub-division rect 

        %% Inter-occlusion prevention 
        
        if n == 1 % other agent will be 2
            other_agent_loc = [targets_xs(2,t) targets_ys(2,t) 1];
        else % other agent will be 1            
            other_agent_loc = [targets_xs(1,t) targets_ys(1,t) 1] ;
        end
                
        map3_h = copy(map3); % map3 object considering the presence of other agent 

        % should do this twice for update 
        map3_h.updateOccupancy(other_agent_loc,1);
        map3_h.updateOccupancy(other_agent_loc,1);

        move_x = [-1,0,1];
        move_y = [-1,0,1];
        move_z = [-1 0 1];
        
        res_stride = 1;
                
        % for clarity, we should inflate the occupancy around the other (but too conservative)
        % agent         
        for i = 1:3
            for j = 1:3
                for k = 1:3
                    for stride = 1:res_stride
                        other_agent_loc_around = other_agent_loc + stride*(1/res) * [move_x(i) move_y(j) move_z(k)];
                        % should do this twice 
                        map3_h.updateOccupancy(other_agent_loc_around,1);
                        map3_h.updateOccupancy(other_agent_loc_around,1);
                    end
                end
            end
        end
        
        %% Contruct inequality condition (polyhydre) from rect region in (azim,elev) space
        
        azim_set = linspace(0,2*pi,N_azim(n)+1);
        azim_set = azim_set(1:end-1);
        
        elev_max = 4*pi/9; elev_min=pi/18;
        elev_set = linspace(elev_min,elev_max,N_elev(n));
        
        max_ray = 6; % maximum allowable tracking distance 
        min_ray =2; % minmum tracking distacne 
        
        ray_cast_res = 1/res;  % ray cast stride distance 
        clustering_delta_r = 1/res/2; % threshold for neigborhood-ness for hit distance 
        
        [DT_set, pinch_bin]=get_multi_depth_DT(map3_h,[target_xs(t) target_ys(t) target_zs(t)],azim_set,elev_set,max_ray,ray_cast_res,clustering_delta_r);
        % DT set update 
                
        % let's save the pinching process                         
        DT_set_t{t} = DT_set; 
        pinch_bin_t{t} = pinch_bin;
        
        figure() % new figure for this time step and this target 
        title_tot = sprintf('%d th target in %d time step',n,t);
        sgtitle(title_tot)
        
        pinch_bin = [0 pinch_bin];
        
        % pinch_bin : increasing order 
        for s = 1:length(DT_set) % length DT_set = length pinch bin
            
            % SDFT drawing 
            subplot(length(pinch_bin)-1,1,s)                
            title_sub = sprintf('%d th pinch ray length for %.2f',s,pinch_bin(s+1)); 
            title(title_sub)
            
            DT = DT_set{s};
            rect_idx = 0;
            % extract rect region 
            if sum(DT) == inf % no hit occured
                N_row = 3; N_col = 2; % in this case, we just assign decent number to divide the region 
                azim_div = floor(linspace(1,N_azim(n),N_row + 1)); elev_div = floor(linspace(1,N_elev(n),N_col+1));
                rects = {}; % rectangle of this pinch step 

                for r = 1:N_row
                    for c = 1: N_col
                        rect.lower = [azim_div(r),elev_div(c)];
                        rect.upper = [azim_div(r+1),elev_div(c+1)];
                        rect.score = inf; 
                        rect_idx = rect_idx + 1;
                        rects{rect_idx} = rect; 
                    end
                end
                
                plot_DT_rectDiv(10*ones(size(DT,1),size(DT,2)),rects); % in case of no hit.. just plane white for DT 
                
                
            else % if there's hit 
                N_rect = 6; % number of recommendation rect 
                rects = rectDiv(DT,N_rect,r_max_stride,c_max_stride,stride_res);                    
                
                plot_DT_rectDiv(DT,rects); 
                
            end  

            % enclosing region of this polyhedra segment (6 surfaces or 5 surfaces in the first pinch)
            % these two pinch determine the fore and back enclosing surface
            
            pinch_prev = pinch_bin(s);
            pinch_cur = pinch_bin(s+1);
            
           for rect_idx = 1:length(rects)
                                
                azim1 = azim_set(rects{rect_idx}.lower(1));
                elev1 = elev_set(rects{rect_idx}.lower(2));
                azim2 = azim_set(rects{rect_idx}.upper(1));
                elev2 = elev_set(rects{rect_idx}.upper(2));
                                               
                % two fore and back surface 
                v = [getVec(azim1,elev1) ; getVec(azim1,elev2) ; getVec(azim2,elev2) ; getVec(azim2,elev1)];


                inner_surf_four_corners = [target_xs(t) target_ys(t) target_zs(t)] + pinch_prev *  v; % 4 x 3
                outer_surf_four_corners = [target_xs(t) target_ys(t) target_zs(t)] + pinch_cur *  v; % 4 x 3
                v_center = cross((mean(outer_surf_four_corners) - outer_surf_four_corners(1,:)),(mean(outer_surf_four_corners) - outer_surf_four_corners(4,:)));
                v_center = v_center / norm(v_center);

                
                % inspection 
%                 figure 
%                 hold on 
%                 scatter3(inner_surf_four_corners(:,1),inner_surf_four_corners(:,2),inner_surf_four_corners(:,3),'bo')
%                 scatter3(outer_surf_four_corners(:,1),outer_surf_four_corners(:,2),outer_surf_four_corners(:,3),'bo')
%                 axis equal
%                 
                
                % enclosing lateral surface of this polyhedra segment                 
                A  = [cross(v(1,:),v(2,:)) ; cross(v(2,:),v(3,:)) ; cross(v(3,:),v(4,:)) ; cross(v(4,:),v(1,:)) ; - v_center ; v_center];
                b = [A(1,:) * inner_surf_four_corners(1,:)' ;...
                    A(2,:) * inner_surf_four_corners(2,:)' ;...
                    A(3,:) * inner_surf_four_corners(3,:)';...
                    A(4,:) * inner_surf_four_corners(4,:)';...
                    A(5,:)*  inner_surf_four_corners(1,:)';...
                    A(6,:)*  outer_surf_four_corners(1,:)'];
                                                                                                
                % update  block                                    
                % remind : s is pinch idx / rect_idx 
                A_set_t{t}{s}{rect_idx} = A;
                b_set_t{t}{s}{rect_idx} = b;                                                                  
                cost_set_t{t}{s}(rect_idx) = 1/rects{rect_idx}.score; % should check the value of score (inf value means no hit occured)
                rect_set_t{t}{s}{rect_idx} = rects{rect_idx}; % rectangle 
                                
            end % rect

        end % rectangle extraction in (azim,elev) space (pinch)
                
    end % time 
                           
    % save for each agent 
     visi_info.A_set = A_set_t;
     visi_info.b_set = b_set_t;
     visi_info.cost_set = cost_set_t;
     visi_info.rect_set = rect_set_t;
     visi_info.DT_t = DT_set_t;
     visi_info.pinch_bin = pinch_bin_t;
     
     % append it 
     visi_info_set{n} = visi_info;
         
end % target 

