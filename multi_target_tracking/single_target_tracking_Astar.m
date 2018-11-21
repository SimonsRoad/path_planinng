%% Description 
% this code simulates a scenario. 
addpath('C:\Users\junbs\Documents\path_planinng\ASAP1','C:\Users\junbs\Documents\path_planinng\multi_target_tracking\');


%% Map setting
map_dim = 20;
lx = 10; ly = 10; % size of map in real coordinate 
res = lx / map_dim;
custom_map=makemap(20); % draw obstacle interactively
% or load the simple example 
load('prob_setting.mat')
%% Generate map occupancy grid object 
map = robotics.OccupancyGrid(flipud(custom_map),1/res);
show(map);
[target_xs,target_ys,tracker]=set_target_tracker; % assign target and tracker
%% Get score map from each target point

vis_cost_set = []; % row : time / col : angle index 
N_azim = 10;
DT_set = [];
for t = 1:length(target_xs) % -1 is due to some mistake in set_target_tracker function 
    target_position = [target_xs(t), target_ys(t) 0]';
    ray_len = norm(target_position(1:2) - tracker);
    ray_lens = ray_len * ones(N_azim,1);
    angles = linspace(0,2*pi,N_azim+1);
    angles(end) = [];
    
    cast_res = zeros(1,N_azim); % 1 for hit and 0 for free rays    t
    collisionPts=map.rayIntersection(target_position,angles,ray_len); % the NaN elements are not hit, 
    collisionIdx=find(~isnan(collisionPts(:,1))); % collided index 
    cast_res(collisionIdx) = 1;
    DT = signed_distance_transform([cast_res cast_res cast_res]); % periodic distance transform 
    DT = DT(N_azim+1:2*N_azim); % extracting 
    DT_set = [DT_set ; DT ];
    vis_cost = max(DT) - DT + 1; 
    vis_cost_set(t,:) = vis_cost;    
end
%% draw  problem settings

% for now, the feasible region for solving discrete path is just rectangle
% (TODO: extension for general affine region)
H = length(target_xs);
x_range = 12;
y_range = 6;
feasible_domain_x = [tracker(1) - x_range/2 tracker(1) + x_range/2];
xl = feasible_domain_x(1);
xu = feasible_domain_x(2);
feasible_domain_y = [tracker(2) - y_range/2  tracker(2)  + y_range/2];
yl = feasible_domain_y(1);
yu = feasible_domain_y(2);

% plot the problem 
show(map)
hold on 
target_path_plot=plot(target_xs,target_ys,'r^-','LineWidth',2);
plot(tracker(1),tracker(2),'ko')
patch([xl xu xu xl],[yl yl yu yu],'red','FaceAlpha',0.1)

% A_sub, b_sub : inequality matrix of each sub division region (sigma Nh  pair)
Nh = zeros(1,H); % we also invesigate number of available regions per each time step 
angles = linspace(0,2*pi,N_azim+1);
S = [];
A_sub  = {};
b_sub = {};
N_vis_show = 3; % shows N largest vis score 
d_ref = norm([target_xs(1) target_ys(1)]' - tracker);

for h = 1:H
    Nk = 0; % initialize number of valid region
        
    for k = 1:N_azim    
        % Bounding lines of the k th pizza segment !
        theta1 = 2*pi/N_azim * (k-1);
        theta2 = 2*pi/N_azim * (k);        
        v1 = [cos(theta1), sin(theta1)]'; 
        v2 = [cos(theta2) , sin(theta2)]'; 
        % If this holds, then one of the two line is in the box 
        if ( is_in_box(v1,[target_xs(h) ; target_ys(h)],[xl xu],[yl yu]) || is_in_box(v2,[target_xs(h) ; target_ys(h)],[xl xu],[yl yu]) )
            if(DT_set(h,k)) % occlusion and collision rejection
                [A,b] = get_ineq_matrix([target_xs(h) ; target_ys(h)],v1,v2);        
                Nk = Nk +1;
                A_sub{h}{Nk} = A; b_sub{h}{Nk}=b; % inequality constraint
                S=[S vis_cost_set(h,k)]; % visbiility cost of the region 
            end
        end          
    end
    
    [sorted_val, indices] = sort(vis_cost_set(h,:));
    goods = indices(1:N_vis_show);
   
    for i = 1:N_vis_show
        good = goods(i);
        draw_circle_sector([target_xs(h) target_ys(h)],2*pi/N_azim * (good-1),2*pi/N_azim * (good),d_ref,'g',1/sorted_val(i)*0.3);        
    end
    
    Nh(h) = Nk; % save the available number of region
end


%% Search map setting for Astar 

search_res = 0.5; 
Xs = xl:search_res:xu;
Ys = yl:search_res:yu;
[x_mesh,y_mesh]  = meshgrid(Xs,Ys);
mesh_row = size(x_mesh,1);
mesh_col = size(y_mesh,2);

% draw generated grid space for A star 

% horizontal line 
for r = 1: mesh_row
    plot([xl xu],[Ys(r) Ys(r)],'k-')
end

% vertical line
for c = 1: mesh_col
    plot([Xs(c) Xs(c)],[yl yu],'k-')
end

% flatten mesh 
x_mesh = reshape(x_mesh,1,[]);
y_mesh = reshape(y_mesh,1,[]);
pnts_grid = [x_mesh ; y_mesh];
[x_mesh,y_mesh]  = meshgrid(Xs,Ys);


%% Graph Construction 
% graph library is shit. have to keep names and points both...
node_name_array = {}; %string
node_loc_array = {}; % actual points (R^2)
node_cost_array={}; % visbility cost vector 

name_vec = {'t0n1'};
loc_vec = tracker;
node_name_array{1} = name_vec;
node_loc_array{1} = loc_vec;

d_max = 3; % allowable interval distance in 1 norm sense 
d_des = 2;

n_grid = length(pnts_grid);
w_v = 10; % visibility weight for optimization 
w_d = 1; % weight for desired distance 

Astar_G = digraph();

elapsed1 =zeros(1,H);
elapsed2 = 0;

% we will update the edge only once not every time 
node1_list = {};
node2_list = {};
weight_list = [];
edge_cnt = 0;

for h = 1: H    
    node_name_array{h+1} = strseq(strcat('t',num2str(h),'n'),1:n_grid);    

    %% Phase 1:  adding nodes for the current timestep
    tic 
    Astar_G=Astar_G.addnode(node_name_array{h+1});    
    node_loc_array{h+1} = pnts_grid;    
    cur_target_pos = [target_xs(h) ; target_ys(h)];     
    diff = pnts_grid - cur_target_pos;    
    grid_angles = atan2(diff(2,:),diff(1,:)); % angles of vectors connecting each pnt on the grid and current target position    
    grid_angles(grid_angles < 0)  = grid_angles(grid_angles < 0)  + 2*pi; 
    grid_segs = ceil(grid_angles/(2*pi/N_azim)); % belonging pizza segments 
    node_cost_array{h} = abs(1./DT_set(h,grid_segs));
    % mapping from grid index (r,c) to flattend index (n)
    grid2flat = @(r,c,N_row) r + N_row * (c-1);
    elapsed1(h)=toc;
    %% Phase 2: adding egdes between two time step     
%     tic 
    for prev_idx = 1: length(node_name_array{h})
                
            % we will find the indices of neighbor hood of loc1 spanned by
            % square box (loc1, d_max)                    
            loc1 = node_loc_array{h}(:,prev_idx);            
            
            r_idx_min_box = max (ceil((loc1(2) - d_max -yl)/ search_res),1);
            c_idx_min_box = max(ceil((loc1(1) - d_max -xl)/ search_res),1);            
            r_idx_max_box = min(floor((loc1(2) + d_max -yl)/ search_res),size(x_mesh,1));
            c_idx_max_box = min(floor((loc1(1) + d_max -xl)/ search_res),size(x_mesh,2));          
            
            box_indices_flat = [];            
            for r_idx_box = r_idx_min_box : r_idx_max_box
                for c_idx_box = c_idx_min_box : c_idx_max_box
                    box_indices_flat=[box_indices_flat grid2flat(r_idx_box,c_idx_box,size(x_mesh,1))]; 
                end
            end            
            
           for current_neightbor_idx = box_indices_flat % this index belongs to the current step
                loc2 = node_loc_array{h + 1}(:,current_neightbor_idx); 
                weight = w_v * node_cost_array{h}(current_neightbor_idx) + norm(loc1-loc2) + w_d * abs(norm(loc2 - [target_xs(h); target_ys(h)]) - d_des);
                edge_cnt = edge_cnt + 1;
                node1_list{edge_cnt} = node_name_array{h}{prev_idx};
                node2_list{edge_cnt} = node_name_array{h+1}{current_neightbor_idx};
                weight_list(edge_cnt) = weight;
%                 Astar_G=Astar_G.addedge(node_name_array{h}{prev_idx},node_name_array{h+1}{current_neightbor_idx},weight);           % this is very slow  
           end            
            % test : is this right? 
%             hold on 
%             plot(node_loc_array{h+1}(1,box_indices_flat),node_loc_array{h+1}(2,box_indices_flat),'ms')
%             plot(loc1(1),loc1(2),'bs')            
%             axis equal                                                    
    end           
%     elapsed2(h)=toc;
end

large_number = 1e+5;
weight_list(weight_list == inf) = large_number;
elapsed2 = 0;
% wrapping the graph 
tic     
for k = 1: length(node_name_array{H+1})
    edge_cnt = edge_cnt + 1;
    node1_list{edge_cnt} = node_name_array{H+1}{k};
    node2_list{edge_cnt} = 'xf';
    weight_list(edge_cnt)= 0.1;
end
Astar_G=Astar_G.addedge(node1_list,node2_list, (weight_list));
elasped2=toc

    %% Phase 4: path solve
        
    tic
    [path_idx,total_cost]=Astar_G.shortestpath('t0n1','xf','Method','auto');    
    elapsed4=toc; 
    
    idx_seq = [];
    for pnt_idx = path_idx
        pnt_idx_convert=cell2mat(pnt_idx);
        idx_seq = [idx_seq str2num(pnt_idx_convert(4:end))];        
    end   
    idx_seq = idx_seq(1:end);    
    path_pnts = [tracker pnts_grid(:,idx_seq(2:end))];
    
   
    %% Analysis Graph and plot 
    hold on     
    path_plot=plot([tracker(1) path_pnts(1,:)],[tracker(2) path_pnts(2,:)],'bs-','LineWidth',2);

%     leg=legend([path_plot,target_path_plot],{'$A^{*}$','target'},'Interpreter','latex','fontsize', 15);

    
 % Should run "single_target_tracking_MILP" if use the below legend 
 
     leg=legend([path_plot,MILP_path_plot1,target_path_plot],{'$A^{*}$','MILP','target'},'Interpreter','latex','fontsize', 15);
%     delete(leg)

%% optional : draw visbility cost map in the grid field for a time step 

fig = figure;
set(fig, 'position', [200, 400, 400, 600] );
for h = 1:H
    subplot(H,1,h);
    plot([tracker(1) path_pnts(1,:)],[tracker(2) path_pnts(2,:)],'bs-','LineWidth',1);
    h_show = h;    
    hold on 
    target_pos=plot(target_xs(h_show),target_ys(h_show),'r^','MarkerSize',10,'MarkerFaceColor','r');    
    plot(path_pnts(1,h+1),path_pnts(2,h+1),'bs','MarkerFaceColor','b','MarkerSize',10)
        view(-10,90)
    for plot_idx = 1:length(pnts_grid)    
        patch([(pnts_grid(1,plot_idx) - search_res/2), (pnts_grid(1,plot_idx) +search_res/2), (pnts_grid(1,plot_idx) +search_res/2), (pnts_grid(1,plot_idx) - search_res/2)]...
            ,[(pnts_grid(2,plot_idx) - search_res/2), (pnts_grid(2,plot_idx) - search_res/2), (pnts_grid(2,plot_idx) +search_res/2) ,(pnts_grid(2,plot_idx) + search_res/2)],...
            'green','FaceAlpha',(1/node_cost_array{h_show}(plot_idx)) /(1/min(node_cost_array{h_show}))*0.5,'EdgeAlpha',0.1 )    
    end

    axis off
    title(sprintf('$ t_{%d}$',h),'Interpreter','latex','FontSize',20)

     axis([min(pnts_grid(1,:)) max(pnts_grid(1,:)) min(pnts_grid(2,:)) max(pnts_grid(2,:))])
end


%% illustrate connectivity 
% eg source node
subplot(H,1,1)
loc1 = path_pnts(:,2);            
% plot(loc1(1),loc1(2),'bs','MarkerFaceColor','b','MarkerSize',20)      

% eg tree node of the root 
subplot(H,1,2)
h = 2;
d_max = 2;
r_idx_min_box = max (ceil((loc1(2) - d_max -yl)/ search_res),1);
c_idx_min_box = max(ceil((loc1(1) - d_max -xl)/ search_res),1);            
r_idx_max_box = min(floor((loc1(2) + d_max -yl)/ search_res),size(x_mesh,1));
c_idx_max_box = min(floor((loc1(1) + d_max -xl)/ search_res),size(x_mesh,2));          

box_indices_flat = [];            
for r_idx_box = r_idx_min_box : r_idx_max_box
    for c_idx_box = c_idx_min_box : c_idx_max_box
        box_indices_flat=[box_indices_flat grid2flat(r_idx_box,c_idx_box,size(x_mesh,1))]; 
    end
end        

hold on 
plot(node_loc_array{2+1}(1,box_indices_flat),node_loc_array{2+1}(2,box_indices_flat),'ms')
axis([min(pnts_grid(1,:)) max(pnts_grid(1,:)) min(pnts_grid(2,:)) max(pnts_grid(2,:))])
axis off    


%% Optional - calculation times 
figure
ax = axes;
res = [1,2,3,4,5];  
Astar_calc1 = [0.01 nan 0.23 nan 9];
Astar_calc2 = [0.0025 nan 0.0063 nan 0.0293];
bar(res,[Astar_calc1;Astar_calc2]','stacked')
ax.YScale = 'log';








