%% Description
% this code simulates a scenario. 

addpath('..\ASAP1','.\','..\plotregion\');
addpath('..\STOMP\')

%% Phase1 : Map setting in 2D projection space

map_dim = 20;
lx = 10; ly = 10; % size of map in real coordinate 
res = lx / map_dim;
% custom_map=makemap(20); % draw obstacle interactively
load('problem_settings.mat')
% Generate map occupancy grid object and path of the two targets

% map = robotics.OccupancyGrid(flipud(custom_map),1/res);
show(map);
% [target1_xs,target1_ys,target2_xs,target2_ys,tracker]=set_target_tracker2; % assign path of two targets and tracker

% packing the target paths (should be same length)
% targets_xs = [target1_xs ; target2_xs];
% targets_ys = [target1_ys ; target2_ys];

% Clustering and give height for each obstacle 
epsilon=1; % neihbor hood bound 
MinPts=5; % minimum grouping index 
[occ_rows,occ_cols] = find(map.occupancyMatrix >= map.OccupiedThreshold);
occ_cells= [occ_rows occ_cols];
IDX=DBSCAN(occ_cells,epsilon,MinPts);
figure
PlotClusterinResult(occ_cells,IDX)
heights = [6,2];

% construct 3D map from 2D ground plan 
map3 = robotics.OccupancyMap3D(map.Resolution);
res = map.Resolution;

pcl = []; % should be N x 3
for idx = 1:max(IDX)
    xy_pnts=map.grid2world(occ_cells(find(IDX==idx),:));
    for r = 1:size(xy_pnts,1)
            zs = 0:1/res:heights(idx);
            for z = zs                
                % point cloud append 
                pcl = [pcl ; [xy_pnts(r,:) z]] ;               
            end
    end              
end

pcl_origin_pose = [ 0 0 0  1 0 0 0];  
max_range = 20;
map3.insertPointCloud(pcl_origin_pose,pcl,max_range);
figure(1)
show(map3)
axis equal
hold on 
N_target = 2; % only two targets will be considered

% path of each target 
target1_xs = targets_xs(1,:); target1_ys = targets_ys(1,:);  targets_zs(1,:) = 0.5*ones(1,length(target1_xs));
target2_xs  = targets_xs(2,:); target2_ys = targets_ys(2,:); targets_zs(2,:) = 0.5*ones(1,length(target1_xs));

plot(target1_xs,target1_ys,'r^-','LineWidth',2)
plot(target2_xs,target2_ys,'r^-','LineWidth',2)

vis_cost_sets = {};

%% Phase2: Get score maps and inequality condition set of each target during a prediction horizon

% parameter for observation 
d_ref = 2;
N_azim = [40,40];
N_elev = [10, 10];

% parameter for sub-division             
N_rect = 10; r_max_stride = 6; c_max_stride = 6; stride_res = 1;                

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
        
        %%  Construct visibility matrix and representation for each rect region 
        
        cast_res = zeros(N_azim(n),N_elev(n)); % 1 for hit and 0 for free rays                   
        target_position = [target_xs(t), target_ys(t) 0]';        
        ray_len = d_ref;
        
        azim_set = linspace(0,2*pi,N_azim(n)+1);
        azim_set = azim_set(1:end-1);
        
        elev_max = 4*pi/9; elev_min=pi/18;
        elev_set = linspace(elev_min,elev_max,N_elev(n));
                
        for azim_idx =1: N_azim(n)
            for elev_idx = 1: N_elev(n)
                cast_res(azim_idx,elev_idx) = rayInsert3D(map3_h,target_position,azim_set(azim_idx),elev_set(elev_idx),ray_len,1/res);                
            end
        end
        
        DT = signed_distance_transform([cast_res ; cast_res ; cast_res]); % periodic distance transform         
        DT = DT(N_azim(n)+1:2*N_azim(n),:);        
                               
        if sum(DT==inf) % in this case, no occlusion occurs, let's just find rect with a predefined number 
            
            DT_t{t} = ones(N_azim(n),N_elev(n));
            
            % this can be adjusted arbitrary 
            N_row = 3; % devide N_row  along row-axis 
            N_col = 2;
            
            d_azim = 2*pi / N_row;
            d_elev = (elev_max - elev_min) / N_col;
            
            
            % convert the (azim,elev) rect into (A,b)
            
            for r = 1:N_row 
                for c = 1:N_col
                    
                    azim1= d_azim * (r - 1) ;
                    elev1 =elev_min + d_elev * (c - 1);
                    azim2 = d_azim * (r);
                    elev2 = elev_min + d_elev * (c);

                    % enclosing region of this polyhedra segment 
                    v = [getVec(azim1,elev1) ; getVec(azim1,elev2) ; getVec(azim2,elev2) ; getVec(azim2,elev1)];
                    A  = [cross(v(1,:),v(2,:)) ; cross(v(2,:),v(3,:)) ; cross(v(3,:),v(4,:)) ; cross(v(4,:),v(1,:))];
                    b = A*[target_xs(t) target_ys(t) target_zs(t)]';
                    
                    
                    
                    % update block 
                    Nt = Nt + 1;                    
                    A_set_t{t}{Nt} = A;
                    b_set_t{t}{Nt} = b;      
                    cost_set_t{t}(Nt) = 0;
                    rect = {};
                    rect.lower = [azim1,elev1];
                    rect.upper = [azim2,elev2];
                    rect_set_t{t}{Nt} = rect;
                    
                end
            end
            
                                                         
        else % some ray hits
            
            DT_t{t} = DT;

            
            % extract some subbox region on the (azim,elev) map 
            rects = rectDiv(DT,N_rect,r_max_stride,c_max_stride,stride_res);
                        
            for rect_idx = 1:length(rects)
                
                
                azim1 = azim_set(rects{rect_idx}.lower(1));
                elev1 = elev_set(rects{rect_idx}.lower(2));
                azim2 = azim_set(rects{rect_idx}.upper(1));
                elev2 = elev_set(rects{rect_idx}.upper(2));
                
                               

                % enclosing region of this polyhedra segment 
                
                v = [getVec(azim1,elev1) ; getVec(azim1,elev2) ; getVec(azim2,elev2) ; getVec(azim2,elev1)];
                A  = [cross(v(1,:),v(2,:)) ; cross(v(2,:),v(3,:)) ; cross(v(3,:),v(4,:)) ; cross(v(4,:),v(1,:))];
                b = A*[target_xs(t) target_ys(t) target_zs(t)]';

                
                
                % update  block                                                
                Nt = Nt + 1;
                A_set_t{t}{Nt} = A;
                b_set_t{t}{Nt} = b;                                                                  
                cost_set_t{t}(Nt) = 1/rects{rect_idx}.score; % should check the value of score 
                rect_set_t{t}{Nt} = rects{rect_idx}; % rectangle 
                                
            end % rect
                                    
        end % if for non-occlusion case 
        
                
        
    end % time 
    
    % save for each agent 
     visi_info.A_set = A_set_t;
     visi_info.b_set = b_set_t;
     visi_info.cost_set = cost_set_t;
     visi_info.rect_set = rect_set_t;
     visi_info.DT_t = DT_t;
     
     % append it 
     visi_info_set{n} = visi_info;
         
end % target 



%% Phase 3: plot the visibility region of each target 



% sphere version 
figure
color_set = [1 0 0;0 0 1]; % for 1st target - red / 2nd target - blue 

show(map3)
target1_zs = ones(1,H);
target2_zs = ones(1,H);

hold on 
plot3(target1_xs,target1_ys,target1_zs,'r^-','LineWidth',2)
plot3(target2_xs,target2_ys,target2_zs,'r^-','LineWidth',2)
% plot3(tracker(1),tracker(2),tracker(3),'mo','MarkerFaceColor','m')
draw_box([xl yl zl],[xu yu zu],'k',0.1)

hold on 
for n = 1: N_target         
        for h = 1:length(target1_xs)                                            
            max_alpha = max(1./visi_info_set{n}.cost_set{h});
            for i = 1: length(visi_info_set{n}.rect_set{h})
                
                
                azim_set = linspace(0,2*pi,N_azim(n)+1);
                azim_set = azim_set(1:end-1);

                elev_max = pi/3; elev_min=pi/8;
                elev_set = linspace(elev_min,elev_max,N_elev(n));

                
                xc = [targets_xs(n,h) targets_ys(n,h) targets_zs(n,h)];
                r = d_ref;
                alpha_val =1/visi_info_set{n}.cost_set{h}(i);          
                
                rect = {};
                rect.lower = [azim_set(visi_info_set{n}.rect_set{h}{i}.lower(1)) elev_set(visi_info_set{n}.rect_set{h}{i}.lower(2)) ];
                rect.upper = [azim_set(visi_info_set{n}.rect_set{h}{i}.upper(1)) elev_set(visi_info_set{n}.rect_set{h}{i}.upper(2)) ];
                
                draw_sphere_sector(xc,r,rect,color_set(n,:),(alpha_val/max_alpha)^2*0.3);
                
            end
        end
                
end

figure
title("distance field sequence")
for n = 1:N_target
    for h = 1:H
        
        subplottight(N_target , H , h + H*(n-1));
        % draw the distance field at time t 
        DT = visi_info_set{n}.DT_t{h};
        DT(DT<0) =0;
        c_set = size(DT,2);
        r_set = size(DT,1);
        max_DT = max(max(DT));

        [ys,xs] = meshgrid(1:c_set,1:r_set);
        
        hold on 
        xlabel('row')
        ylabel('col')

        for r = 1:r_set
            for c = 1: c_set

                x1 = xs(r,c)-0.5;
                y1 = ys(r,c)-0.5;
                x2 = xs(r,c) + 0.5;
                y2 = ys(r,c) + 0.5;

                intensity = DT(r,c) / max_DT;        
                patch([x1 x1 x2 x2],[y1 y2 y2 y1],intensity*ones(1,3))
            end
        end
        
        null_matrix = DT <= 1;

        % boundary detection
        [boundary_idx]=bwboundaries(null_matrix);
        boundary_rc = [];    
        for idx=1:length(boundary_idx)
            boundary_rc = [boundary_rc; boundary_idx{idx}];
        end


        for i = 1:length(boundary_rc)
            plot(boundary_rc(i,1),boundary_rc(i,2),'rs','MarkerSize',4);    
        end
        
        
        rects=rectDiv(DT,N_rect,r_max_stride,c_max_stride,stride_res);

        
        % draw rect 
        
        hold on 
        
        for i = 1:length(rects)


                x1 = rects{i}.lower(1);
                y1 = rects{i}.lower(2);
                x2 = rects{i}.upper(1);
                y2 = rects{i}.upper(2);

                patch([x1 x1 x2 x2],[y1 y2 y2 y1],ones(1,3),'FaceAlpha',0.1,'EdgeColor','g','LineWidth',3)

        end

                axis ([0 r_set 0 c_set])
                axis equal

    end
end

hold off 

%% Phase 4 : plot feasible search region (should include all the points on the target paths )

% for now, the feasible region (search space) for solving discrete path is just rectangle

H = length(target_xs);

% height of initial pose of tracker 
margin = 3; 

tracker = [tracker(1) ; tracker(2) ; 2]; 

% domain 
domain_x =[min([tracker(1) reshape(targets_xs,1,[])]) max([tracker(1) reshape(targets_xs,1,[])])];
domain_y =[min([tracker(2) reshape(targets_ys,1,[])]) max([tracker(2) reshape(targets_ys,1,[])])];
domain_z =[min([tracker(3) reshape(targets_zs,1,[])]) max([tracker(3) reshape(targets_zs,1,[])])];

xl = domain_x(1) - margin;
xu = domain_x(2) + margin;

yl = domain_y(1) - margin;
yu = domain_y(2) +margin;

zl = domain_z(1) - margin;
zu = domain_z(2) + margin;



% plot the serach region 
figure
show(map3)
target1_zs = ones(1,H);
target2_zs = ones(1,H);

hold on 
plot3(target1_xs,target1_ys,target1_zs,'r^-','LineWidth',2)
plot3(target2_xs,target2_ys,target2_zs,'r^-','LineWidth',2)
plot3(tracker(1),tracker(2),tracker(3),'mo','MarkerFaceColor','m')
draw_box([xl yl zl],[xu yu zu],'k',0.1)



% affine version 
color_set = [1 0 0;0 0 1]; % for 1st target - red / 2nd target - blue 

for n = 1: N_target         
        for h = 1:length(target1_xs)                                            
            max_alpha = max(1./visi_info_set{n}.cost_set{h});
            for i = 1: length(visi_info_set{n}.rect_set{h})
                 alpha_val =1/visi_info_set{n}.cost_set{h}(i);          
                
                plotregion(-visi_info_set{n}.A_set{h}{i},-visi_info_set{n}.b_set{h}{i},[xl yl zl],[xu yu zu],color_set(n,:),(alpha_val/max_alpha)^2*0.3);              
                                
            end
        end
                
end




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
    
    % we reject the blind region 
    rel_dist = norm([target1_xs(h) target1_ys(h)] - [target2_xs(h) target2_ys(h)]); % distance between the target 
    blind_height = rel_dist/2/tan(FOV/2);
    
    
    
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
%% Plot faesible segment
%%
figure(1)
for h =1 :H 
    subplot(2,H/2,h)                
        
        show(map3)
        hold on 
        plot(target1_xs,target1_ys,'r^-','LineWidth',2)
       
        plot(target1_xs(h),target1_ys(h),'rs','LineWidth',3,'MarkerSize',10)

        plot(target2_xs,target2_ys,'r^-','LineWidth',2)
        
        plot(target2_xs(h),target2_ys(h),'rs','LineWidth',3,'MarkerSize',10)

        plot3(tracker(1),tracker(2),tracker(3),'mo','MarkerFaceColor','m')
        draw_box([xl yl zl],[xu yu zu],'k',0.1)
        
        % FOV blind region plot        
%         is_in_blind3([target1_xs(h) target1_ys(h) height],[target2_xs(h) target2_ys(h) height],FOV,[],1);
        
        for k = 1:length(vis_cost_set{h})                
            alpha = 1/vis_cost_set{h}(k);   
            alpha_max = max(1./vis_cost_set{h});
            alpha_min = min(1./vis_cost_set{h});
            [r,g,b]  = getRGB(alpha,alpha_min,alpha_max,1);
            plotregion(-A_div{h}{k} ,-b_div{h}{k} ,[xl yl zl]',[xu yu zu]',[r,g,b],0.5);
            plot(c_div{h}{k}(1),c_div{h}{k}(2),'ks','MarkerSize',1.5,'MarkerFaceColor','k');            
        end
        
        axis([xl xu yl yu zl zu])
        axis equal
        
end
%% Astar for the path of region segmentsd

node_name_array = {}; %string -> this will be used in matalb graph library 
node_idx_array={}; % segment index (what time? and what (A,b)?)

name_vec = {'t0n1'};
loc_vec = tracker;
node_name_array{1} = name_vec; % initialize the node name array

w_v = 10; % visibility weight for optimization 
w_d = 1; % weight for desired distance 
d_max = 4; % allowable connecting distane btw "center of region"
Astar_G = digraph();

% for update egde only once at the final phase 
node1_list = {};
node2_list = {};
weight_list = [];
edge_cnt = 0;

for h = 1:H
        % phase1 : add node at this future step
        node_name_array{h+1} = strseq(strcat('t',num2str(h),'n'),1:length(vis_cost_set{h}));    
        
        Astar_G=Astar_G.addnode(node_name_array{h+1});    
        cur_target_pos = [target_xs(h) ; target_ys(h)];     % target position at this time step 
        
        % phase2: connect edges with previus layer
        for idx1 = 1:length(node_name_array{h}) % previous step
            for idx2 = 1:length(node_name_array{h+1}) % current step                
                % would-be node 
                cur_observ_pnt = c_div{h}{idx2};                 
                if h ~= 1
                    prev_observ_pnt= c_div{h-1}{idx1};
                else
                    prev_observ_pnt = tracker;
                end
                travel_distance = norm([prev_observ_pnt(1) - cur_observ_pnt(1),prev_observ_pnt(2) - cur_observ_pnt(2)]);
                if (travel_distance < d_max) % for sparsity 
                    edge_cnt = edge_cnt + 1;
                    vis_cost = vis_cost_set{h}{idx2};                
                    deviation_d_ref = (d_ref - norm([cur_observ_pnt(1)-cur_target_pos(1),cur_observ_pnt(2)-cur_target_pos(2)]))^2;       
                    weight = w_v*vis_cost + w_d*deviation_d_ref + travel_distance;
                    % add the two connecting nodes with the weight 
                    node1_list{edge_cnt} = node_name_array{h}{idx1};
                    node2_list{edge_cnt} = node_name_array{h+1}{idx2};                    
                    weight_list(edge_cnt) = weight;                    
                end
            end
        end                                    
end

 % phase3 : graph wrapping 
 for k = 1: length(node_name_array{H+1})
    edge_cnt = edge_cnt + 1;
    node1_list{edge_cnt} = node_name_array{H+1}{k};
    node2_list{edge_cnt} = 'xf';
    weight_list(edge_cnt)= 0.1;
end
 
Astar_G=Astar_G.addedge(node1_list,node2_list, (weight_list));
[path_idx,total_cost]=Astar_G.shortestpath('t0n1','xf','Method','auto');
%% plotting the planned path
%%
figure(1)
hold on 
idx_seq = [];
for pnt_idx = path_idx
    pnt_idx_convert=cell2mat(pnt_idx);
    idx_seq = [idx_seq str2num(pnt_idx_convert(4:end))];        
end   
idx_seq = idx_seq(2:end);

conv_hull = {};

waypoint_polygon_seq = {};
corridor_polygon_seq = {};


for h = 1:H    
    subplot(2,2,h)
    plotregion(-A_div{h}{idx_seq(h)} ,-b_div{h}{idx_seq(h)} ,[xl yl]',[xu yu]',[1,0,1],0.5);
    
    % 2D version 
%     waypoint_polygon_seq{h}.A = A_div{h}{idx_seq(h)};
%     waypoint_polygon_seq{h}.b = b_div{h}{idx_seq(h)};
    
    % 3D version 
    waypoint_polygon_seq{h}.A = [A_div{h}{idx_seq(h)} zeros(size(A_div{h}{idx_seq(h)},1),1)];
    waypoint_polygon_seq{h}.b =[b_div{h}{idx_seq(h)}];
    
       
    
    if h ==1 
        vert1 = tracker';
    else
        vert1 = v_div{h-1}{idx_seq(h-1)};        
    end    
    vert2 = v_div{h}{idx_seq(h)};         
    vert = [vert1 ; vert2];       
    K = convhull(vert(:,1), vert(:,2));              
    patch(vert(K,1), vert(K,2),[0 0 0],'EdgeColor','g','LineWidth',4,'FaceAlpha',0.1);     
    
    [A_corr,b_corr]=vert2con(vert(K,:));
    % corridor connecting each waypoint polygon 
    
%     corridor_polygon_seq{h}.A =[A_corr] ;
%     corridor_polygon_seq{h}.b = [b_corr]; 
    
    corridor_polygon_seq{h}.A =[A_corr zeros(size(A_corr,1),1)] ;
    corridor_polygon_seq{h}.b = b_corr;        
    axis([0 10 0 10])
end

% save('polygon_seq','waypoint_polygon_seq','corridor_polygon_seq');
%% Generation of smooth path (currently, the convex hull is assumed to be collision free, additional modification required)
%%
ts= [0 1 2 3 4];

X0 = [tracker;0];
Xdot0 = zeros(3,1);
Xddot0 = zeros(3,1);


% smooth path generation in the corrideor 
[pxs,pys,pzs]=min_jerk_ineq(ts,X0,Xdot0,Xddot0,waypoint_polygon_seq,corridor_polygon_seq);

% draw path 
figure(1) 
for h = 1:H
    subplot(2,2,h)
    hold on
    plot_poly_spline(ts,reshape(pxs,[],1),reshape(pys,[],1),reshape(pzs,[],1))
    axis equal
end