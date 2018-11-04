%% Description 
% this code simulates a scenario. 
addpath('C:\Users\junbs\Documents\path_planinng\ASAP1','C:\Users\junbs\Documents\path_planinng\multi_target_tracking\');


%% Map setting
map_dim = 20;
lx = 10; ly = 10; % size of map in real coordinate 
res = lx / map_dim;
custom_map=makemap(20); % draw obstacle interactively

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
%% Feasible region convex division for LP problem 

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
plot(target_xs,target_ys,'r*')
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

%% Generation of optimal sequence : everything was converted into LP (refer lab note)


%%%
% Optimization variables 
% X = [x1 y1 x2 y2 ... xH yH  ||  d1x d1y ... dHx dHy || j1x j1y ... jHx jHy || z_1,1 z_1,2 ... z_1,N1 | ..| z_H,1 ... z_H,N_H 
%  (X+, X-), Ax, (Y+, Y-), Ay, D ]
% w_v: weight for visibility 
%%%


%%%
% Parameters
% w_j : weight for jerk 
% w_v: weight for visibility 
%%%

H  = length(target_xs) ; % total horizon 
N_var = 2*H + 2*H + 2*(H-2) + length(S) + 7*H; % x,y,dx,dy,jx,jy,z, (X+, X-), (Y+, Y-), Ax, Ay, D 
w_j = 1;
w_v = 50;
w_d = 2; % weight for desired distance

% objective function : sum of travel (1 norm), sum of jerk, sum of visibility cost  
d_des_f_idx1 = 2*H + 2*H + 2*(H-2) + length(S) + 7:7:N_var;
f = [zeros(1,2*H),  ones(1,2*H) ,  w_j * ones(1,2*(H-2)), w_v*S zeros(1,7*H)];
f(d_des_f_idx1 ) = w_d;

% equality constraint
Aeq = zeros(H,N_var);
insert_idx = 2*H + 2*H  + 2*(H-2) + 1;
    
for h = 1:H
    Aeq(h,insert_idx:insert_idx+Nh(h)-1) = ones(1,Nh(h));
    insert_idx = insert_idx+Nh(h)  ;
end
beq = ones(H,1);
   

% inequality 1 : travel distance auxiliary variables
Aineq1 = zeros(4*H,N_var); bineq1 = zeros(4*H,1);
insert_mat_xy = [-1 0 ; 1 0 ; 0 -1 ; 0 1];
insert_mat_d = [-1 0 ; -1 0; 0 -1; 0 -1];
insert_row = 1;
insert_col = 1;


for h = 1:H
    if h == 1
        Aineq1(insert_row:insert_row+3,insert_col:insert_col + 1) = insert_mat_xy; bineq1(insert_row:insert_row+3) = [-tracker(1) tracker(1) -tracker(2) tracker(2)]';
        Aineq1(insert_row:insert_row+3,insert_col + 2*H  : insert_col + 2*H +1 ) = insert_mat_d;
    else
        Aineq1(insert_row:insert_row+3,insert_col-2:insert_col+1) = [-insert_mat_xy insert_mat_xy];
        Aineq1(insert_row:insert_row+3,insert_col + 2*H : insert_col + 2*H + 1) = insert_mat_d;        
    end    
    insert_row = insert_row + 4;
    insert_col = insert_col + 2;
end



% inequality 2 : jerk auxiliary variables

insert_row = 1;
insert_col = 1;
Aineq2 = zeros(4*(H-2),N_var);
bineq2 = zeros(4*(H-2),1);
insert_mat_xy = [-1 0 ; 1 0 ; 0 -1 ; 0 1];
insert_mat_xy = [-insert_mat_xy 3*insert_mat_xy -3*insert_mat_xy insert_mat_xy];

for h = 1:H - 2
    if h == 1
        Aineq2(insert_row:insert_row+3,insert_col:insert_col + 5) = insert_mat_xy(:,3:end); bineq2(insert_row:insert_row+3) = [-tracker(1) tracker(1) -tracker(2) tracker(2)]';
        Aineq2(insert_row:insert_row+3,insert_col + 4*H   : insert_col + 4*H +1  ) = insert_mat_d;
    else
        Aineq2(insert_row:insert_row+3,insert_col-2:insert_col+5) = insert_mat_xy;
        Aineq2(insert_row:insert_row+3,insert_col + 4*H  : insert_col + 4*H + 1 ) = insert_mat_d;        
    end    
    insert_row = insert_row + 4;
    insert_col = insert_col + 2;
end


% inequality 3 : sub division inequality (sum of Nh (h= 1... H) many constraints)
M = 1e+2; % some large number (big enough but not too be big)

dim = 2; % currently 2D
Aineq3 = zeros(dim * sum(Nh),N_var);
bineq3 = M*ones(dim * sum(Nh),1);

insert_blk_row = 1;
for h = 1:H 
    for k = 1: Nh(h)
        Aineq3(2*(insert_blk_row-1)+1:2*(insert_blk_row),2*(h-1)+1:2*(h)) = A_sub{h}{k};
        Aineq3(2*(insert_blk_row-1)+1:2*(insert_blk_row), 6*H-4 + (insert_blk_row)) = -b_sub{h}{k} + M; 
        insert_blk_row = insert_blk_row + 1;
    end   
end

% inequality 4: interval distance limit (1 norm) 
d_max = 3;
Aineq4 = zeros(H,N_var); bineq4 = d_max * ones(H,1);
for h = 1:H
    Aineq4(h,2*H + 2*(h-1) + 1 : 2*H + 2*h) = ones(1,2);     
end


% inequality 5: maintaining desired distance  

d_des = 3;

Aineq5 = zeros(14*H,N_var); bineq5 = zeros(14*H,1);
Aeq5 = zeros(2*H,N_var); beq5 = zeros(2*H,1);

until_this=2*H + 2*H + 2*(H-2) + length(S);
for h = 1 : H 
    % var : until this ++ X+, X-, Y+, Y-, Ax, Ay, D  ++ X, Y  
    xc = target_xs(h); yc = target_ys(h);
    Lx = xl - xc; Ux = xu - xc;
    Ly = yl - yc; Uy = yu - yc;
    
    Aineq5_blk = [
    -1 0 0 0 max(0,Lx) 0 0; 
    1 0 0 0 -max(0,Ux) 0 0;
    0 -1 0 0 -max(0,-Ux) 0 0;
    0 1 0 0 max(0,-Lx) 0 0 ;
    0 0 -1 0 0 max(0,Ly) 0;
    0 0 1 0 0 -max(0,Uy) 0;
    0 0 0 -1 0 -max(0,-Uy) 0;
    0 0 0 1 0 max(0,-Ly)  0;
    -1 -1 -1 -1 0 0 -1;
    1 1 1 1 0 0 -1;
    0 0 0 0 -1 0 0;
    0 0 0 0 1 0 0 ;
    0 0 0 0 0 -1 0;
    0 0 0 0 0 1 0 
    ];

  bineq5_blk = [0 0 -max(0,-Ux) max(0,-Lx) 0 0 -max(0,-Uy) max(0,-Ly) -d_des d_des -((abs(Lx)+Lx)/(2*abs(Lx))) ((abs(Ux)+Ux)/(2*abs(Ux))) -((abs(Ly)+Ly)/(2*abs(Ly))) ((abs(Uy)+Uy)/(2*abs(Uy)))]';
  
  Aineq5( 14*(h-1)+1 : 14*h , until_this + 7*(h-1) +1 : until_this + 7*(h) ) = Aineq5_blk;
  bineq5( 14*(h-1)+1 : 14*h ) = bineq5_blk;
    
  Aeq5(2*(h-1)+1 : 2*h, 2*(h-1)+1 : 2*h) = eye(2); 
  Aeq5(2*(h-1)+1 : 2*h, until_this + 7*(h-1)+1 :until_this + 7*(h-1) + 4) = [-1 1 0 0; 0 0 -1 1]; 
  beq5(2*(h-1)+1 : 2*h) = [target_xs(h) ; target_ys(h)];        
end


% optimization 
intcon1 = 6*H -4 + 1 : N_var;
intcon2 = until_this + 5 : 7 : N_var;
intcon3 = until_this + 6 : 7: N_var;

intcon = [intcon1 intcon2 intcon3];

lbs = -inf * ones(N_var,1); lbs(intcon) = 0; lbs(1:2:2*H) = xl;  lbs(2:2:2*H) = yl;
ubs = inf * ones(N_var,1); ubs(intcon) = 1; ubs(1:2:2*H) = xu; ubs(2:2:2*H) = yu;
tic
sol = intlinprog(f,intcon,[Aineq1 ; Aineq2;  Aineq3 ; Aineq4 ;Aineq5],[bineq1; bineq2; bineq3; bineq4; bineq5],[Aeq; Aeq5],[beq; beq5],lbs,ubs);
toc

%% Anaylsis

% variables parsing 
waypoints_x = sol(1:2:2*H);
waypoints_y = sol(2:2:2*H);
plot(waypoints_x,waypoints_y,'bs-','LineWidth',1)
plot([tracker(1) waypoints_x(1)],[tracker(2) waypoints_y(1)],'bs-','LineWidth',1);

select_region_seq = zeros(1,H);
inspect_idx = 6*H -4 + 1;

% for h = 1:H
%     inspect = sol(inspect_idx : inspect_idx + Nh(h) -1);    
%     select_region_seq(h) = find(uint8(inspect)==1);
%     inspect_idx = inspect_idx + Nh(h);   
% end


%% %% Comparison with A star 

%% Search map setting for Astar 
search_res = 1; 
Xs = xl:search_res:xu;
Ys = yl:search_res:yu;
[x_mesh,y_mesh]  = meshgrid(Xs,Ys);
mesh_row = size(x_mesh,1);
mesh_col = size(y_mesh,2);
% draw current problem settings 

%% 

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
plot(target_xs,target_ys,'r*')
plot(tracker(1),tracker(2),'ko', 'MarkerFaceColor','b','MarkerSize',10);
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
G = digraph();
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
w_v = 5; % visibility weight for optimization 
w_d = 1; % weight for desired distance 

%% Graph construction 
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
Astar_G=Astar_G.addedge(node1_list,node2_list, (weight_list));

% Phase 3 : wrapping the graph 
for idx = 1:length(node_name_array{H+1})        
    Astar_G = Astar_G.addedge(node_name_array{H+1}(idx),'xf',0.1);
end

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
    
          
    %% Analysis Graph plot 
    hold on     
    path_plot=plot([tracker(1) path_pnts(1,:)],[tracker(2) path_pnts(2,:)],'bs-','LineWidth',2);
    delete(path_plot)
%     h= plot(Astar_G);
%     highlight(hh,path_idx,'NodeColor','r','EdgeColor','r','LineWidth',3,'MarkerSize',3)
    
    
    
    
    
    
    
    
    

%% optional : draw visbility cost map in the grid field for a time step 


figure
for h = 1:H
    subplot(H,1,h)
    h_show = h;
    hold on 
    plot(target_xs(h_show),target_ys(h_show),'r*')
    for plot_idx = 1:length(pnts_grid)    
        patch([(pnts_grid(1,plot_idx) - search_res/2), (pnts_grid(1,plot_idx) +search_res/2), (pnts_grid(1,plot_idx) +search_res/2), (pnts_grid(1,plot_idx) - search_res/2)]...
            ,[(pnts_grid(2,plot_idx) - search_res/2), (pnts_grid(2,plot_idx) - search_res/2), (pnts_grid(2,plot_idx) +search_res/2) ,(pnts_grid(2,plot_idx) + search_res/2)],...
            'green','FaceAlpha',(1/node_cost_array{h_show}(plot_idx)) /(1/min(node_cost_array{h_show}))*0.5 )    
    end

end









