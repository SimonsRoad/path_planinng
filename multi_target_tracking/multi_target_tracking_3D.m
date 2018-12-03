%% Description
% this code simulates a scenario. 
%% desktop version

addpath('..\ASAP1','.\','..\plotregion\');
addpath('..\STOMP\')
%% laptop version
% addpath('C:\Users\junbs\Documents\path_planinng\ASAP1','C:\Users\junbs\Documents\path_planinng\multi_target_tracking\','C:\Users\junbs\Documents\path_planinng\plotregion\');
%% Map setting in 2D projection space
%%
map_dim = 20;
lx = 10; ly = 10; % size of map in real coordinate 
res = lx / map_dim;
% custom_map=makemap(20); % draw obstacle interactively
load('problem_settings.mat')
%% Generate map occupancy grid object and path of the two targets

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
show(map3)
axis equal
hold on 
N_target = 2; % only two targets will be considered
% path of each target 
target1_xs = targets_xs(1,:); target1_ys = targets_ys(1,:);
target2_xs  = targets_xs(2,:); target2_ys = targets_ys(2,:);

plot(target1_xs,target1_ys,'r^-','LineWidth',2)
plot(target2_xs,target2_ys,'r^-','LineWidth',2)

vis_cost_sets = {};

%% Get score maps of each target during a prediction horizon
d_ref = 2;
N_azim = [10,10];
N_elev = [6, 6];

for n = 1:N_target 
    target_xs = targets_xs(n,:);
    target_ys = targets_ys(n,:);
    
    vis_cost_set = {}; % 1 : time / 2: azim index / 3: elev index  
    DT_set = {};
    for t = 1:length(target_xs) % -1 is due to some mistake in set_target_tracker function 

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
        
        %%  Construct visibility matrix 
        
        cast_res = zeros(N_azim(n),N_elev(n)); % 1 for hit and 0 for free rays                   
        target_position = [target_xs(t), target_ys(t) 0]';        
        ray_len = d_ref;
        
        azim_set = linspace(0,2*pi,N_azim(n)+1);
        azim_set = azim_set(1:end-1);
        elev_set = linspace(pi/8,pi/3,N_elev(n));
                
        for azim_idx =1: N_azim(n)
            for elev_idx = 1: N_elev(n)
                cast_res(azim_idx,elev_idx) = rayInsert3D(map3_h,target_position,azim_set(azim_idx),elev_set(elev_idx),ray_len,1/res);                
            end
        end
        
        DT = signed_distance_transform([cast_res ; cast_res ; cast_res]); % periodic distance transform         
        DT = DT(N_azim(n)+1:2*N_azim(n),:);        
        DT_set{t} = DT ;    % save with respect to time 
        
        if sum(DT==inf) % in this case, no occlusion occurs
            vis_cost = ones(N_azim(n),N_elev(n));
            vis_cost_set{t} = vis_cost ;
        else % some ray hits
            
            % %% Two version of cost map %%%
            vis_cost = max(DT) - DT + 1;         
            vis_cost = abs(1./DT); 
            vis_cost_set{t} = vis_cost;
        end
            
    end % time 
    
    % save for each agent 
    vis_cost_sets{n} = vis_cost_set;
    DT_sets{n} = DT_set;
    
end % target 

%% Phase 0: plot the visibility score 

[elev_grid,azim_grid]=meshgrid(elev_set,azim_set);
azim_grid = reshape(azim_grid,1,[]);
elev_grid = reshape(elev_grid,1,[]);
color_set = [1 0 0;0 0 1]; % for 1st target - red / 2nd target - blue 

for n = 1: N_target         
        for h = 1:length(target1_xs)        
            
            if (sum(sum(vis_cost_sets{n}{h})) ~= numel(vis_cost_sets{n}{h})) % then, every bearing direction is just ok             
                N_vis_show = 5;
            else
                N_vis_show = numel(vis_cost_sets{n}{h});
            end
            
            [sorted_val, indices] = sort(reshape(vis_cost_sets{n}{h},1,[]));
            
            max_alpha = max(1./sorted_val);
            
            goods = indices(1:N_vis_show);  
            
            for i = 1:N_vis_show
                good = goods(i);
                alpha_val = 1./sorted_val(i);
                xc = [targets_xs(n,h) targets_ys(n,h) 0.5];
                r = d_ref;
                
                d_azim = azim_set(2) - azim_set(1);                                
                d_elev = elev_set(2) - elev_set(1);
                hold on 
                draw_sphere_sector(xc,r,azim_grid(good) - d_azim/2,azim_grid(good) + d_azim/2,...
                    elev_grid(good) -d_elev/2, elev_grid(good) + d_elev/2, color_set(n,:),alpha_val/max_alpha * 0.3)
%                 draw_circle_sector(xc,r,2*pi/N_azim_set(n) * (good-1),2*pi/N_azim_set(n) * (good),d_ref/2,color_set{n},alpha_val*0.3);        
            end
        end
end
    
hold off 
%% Phase1 : feasible search region 

% for now, the feasible region (search space) for solving discrete path is just rectangle

H = length(target_xs);
x_range = 12;
y_range = 10;
z_range = 8;

% height of initial pose 
tracker = [tracker ; 2]; 

feasible_domain_x = [tracker(1) - x_range/2 tracker(1) + x_range/2];
xl = feasible_domain_x(1);
xu = feasible_domain_x(2);

feasible_domain_y = [tracker(2) - y_range/2  tracker(2)  + y_range/2];
yl = feasible_domain_y(1);
yu = feasible_domain_y(2);

feasible_domain_z = [tracker(3) - z_range/2  tracker(3)  + z_range/2];
zl = feasible_domain_z(1);
zu = feasible_domain_z(2);


search_spec.x_dom = [xl,xu];
search_spec.y_dom = [yl yu];
search_spec.z_dom = [zl zu];


% plot the serach region 
show(map3)
target1_zs = ones(1,H);
target2_zs = ones(1,H);

hold on 
plot3(target1_xs,target1_ys,target1_zs,'r^-','LineWidth',2)
plot3(target2_xs,target2_ys,target2_zs,'r^-','LineWidth',2)
plot3(tracker(1),tracker(2),tracker(3),'mo','MarkerFaceColor','m')
draw_box([xl yl zl],[xu yu zu],'k',0.1)

%% Phase2: divided region 

color_set = {'g','b'};
visi_info_set = {};

% let's investigate inequality matrix for each time step 
for n = 1:N_target
    % target path 
    target_xs = targets_xs(n,:);
    target_ys = targets_ys(n,:);
    
    % A_sub, b_sub : inequality matrix of each sub division region (only available region)
    Nh = zeros(1,H); % we also invesigate number of available regions per each time step 
    angles = linspace(0,2*pi,N_azim_set(n)+1);
    S = []; % flattened visibility score for easy computation in optimization     
    S_h = {}; % sorted way, this structure has same order  
    A_sub  = {};
    b_sub = {};
    
    for h = 1:H
        Nk = 0; % initialize number of valid region
        for k = 1:N_azim_set(n)    
            % Bounding lines of the k th pizza segment !
            theta1 = 2*pi/N_azim_set(n) * (k-1);
            theta2 = 2*pi/N_azim_set(n) * (k);        
            v1 = [cos(theta1), sin(theta1)]'; 
            v2 = [cos(theta2) , sin(theta2)]'; 
            % If this holds, then one of the two line is in the box 
            if ( is_in_box(v1,[target_xs(h) ; target_ys(h)],[xl xu],[yl yu]) || is_in_box(v2,[target_xs(h) ; target_ys(h)],[xl xu],[yl yu]) )
                if(DT_sets{n}(h,k) && DT_sets{n}(h,min(k+1,N_azim_set(n)))) % occlusion and collision rejection
                    [A,b] = get_ineq_matrix([target_xs(h) ; target_ys(h)],v1,v2);        
                    Nk = Nk +1;
                    A_sub{h}{Nk} = A; b_sub{h}{Nk}=b; % inequality constraint                    
                    S=[S vis_cost_sets{n}(h,k)]; % visbiility cost of the region 
                    S_h{h}{Nk} = vis_cost_sets{n}(h,k); % 
                end
            end          
        end


        Nh(h) = Nk; % save the available number of region
    end

    visi_info.Nh = Nh;
    visi_info.A_sub = A_sub;
    visi_info.b_sub = b_sub; 
    visi_info.S_h = S_h;   
    visi_info_set{n} = visi_info; % save the visibility information 
    
end
%% Phase2 : Combination for divided regions of each agent - indexing
%%
% intersection region of the two pizza corresponding to each target
FOV = 160 * pi/180;
A_div = {};
b_div = {};
c_div = {}; % center of each convex polygorn 
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
    for i = 1:visi_info_set{1}.Nh(h)
        for j = 1:visi_info_set{2}.Nh(h)
            % feasibility test for the intersection region of the two
            % pizzas
            
            Ai = visi_info_set{1}.A_sub{h}{i};
            bi = visi_info_set{1}.b_sub{h}{i};
            vis_cost1 = visi_info_set{1}.S_h{h}{i};
            
            Aj = visi_info_set{2}.A_sub{h}{j};
            bj = visi_info_set{2}.b_sub{h}{j};
            vis_cost2 = visi_info_set{2}.S_h{h}{j};
                              
            A_intsec = [Ai ; Aj];
            b_intsec = [bi ; bj];
            
            A_bound = [1 0 ; -1 0 ; 0 1; 0 -1 ];
            b_bound = [xu ; -xl ; yu ; -yl];
            
            [~,~,flag]=linprog([],[Ai ; Aj ],[bi ; bj] ,[],[],[xl yl],[xu yu]);
            
%             [~,~,flag]=linprog([],[Ai ; Aj ; A_bound],[bi ; bj; b_bounds]);
            
            if (flag ~= -2) % feasibility test pass
               % let's keep this region for now 
               % let's investigate the FOV constraint 
               vertices = con2vert([A_intsec; A_bound],[b_intsec ; b_bound]); % the vertices of this region   
               vertices = vertices + 0.1 * (mean(vertices) - vertices);
               
               % we reject the segment if any of them is included in the
               % bline region                
               if (sum(is_in_blind([targets_xs(1,h) targets_ys(1,h)],[targets_xs(2,h) targets_ys(2,h)],FOV,vertices')) == 0)               
                   
                   Nk = Nk + 1;                     
                   A_div{h}{Nk} = A_intsec;
                   b_div{h}{Nk} = b_intsec; 
                   v_div{h}{Nk} =  vertices;
                   c_div{h}{Nk} = mean(vertices); % center of each segment                

                   
                   vis_cost_set{h}{Nk} =  vis_cost1 + ratio * vis_cost2; % let's assign the visibility cost to here    
               end
            end
            
        end        
    end
end
%% Plot faesible segment
%%
figure(1)
for h =1 :H 
    subplot(2,H/2,h)                
        show(map)
        hold on 
        plot(target1_xs,target1_ys,'r^-','LineWidth',2)
       
        plot(target1_xs(h),target1_ys(h),'rs','LineWidth',3,'MarkerSize',10)

        plot(target2_xs,target2_ys,'r^-','LineWidth',2)
        
        plot(target2_xs(h),target2_ys(h),'rs','LineWidth',3,'MarkerSize',10)

        plot(tracker(1),tracker(2),'mo','MarkerFaceColor','m')
        patch([xl xu xu xl],[yl yl yu yu],'w','FaceAlpha',0.1)
        
        % FOV blind region plot        
        [~,A_blind,b_blind] = is_in_blind([target1_xs(h) target1_ys(h)],[target2_xs(h) target2_ys(h)],FOV,[]);
        plotregion(-A_blind ,-b_blind ,[xl yl]',[xu yu]',[0,0,0]);

        
    for k = 1:length(vis_cost_set{h})                
        alpha = 1/vis_cost_set{h}{k};     % this might be inf    
        [r,g,b]  = getRGB(vis_cost_set{h}{k},10,1);
        plotregion(-A_div{h}{k} ,-b_div{h}{k} ,[xl yl]',[xu yu]',[r,g,b],alpha);
        plot(c_div{h}{k}(1),c_div{h}{k}(2),'ks','MarkerSize',1.5,'MarkerFaceColor','k');
    end
    axis([0 10 0 10])
end
%% Astar for the path of region segmentsd
%%
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