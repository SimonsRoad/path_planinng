%% Description 
% this code simulates a scenario. 
addpath('C:\Users\junbs\Documents\path_planinng\ASAP1','C:\Users\junbs\Documents\path_planinng\multi_target_tracking\');


%% Map setting
map_dim = 20;
lx = 10; ly = 10; % size of map in real coordinate 
res = lx / map_dim;
custom_map=makemap(20); % draw obstacle interactively

%% Generate map occupancy grid object  and path of the two targets
map = robotics.OccupancyGrid(flipud(custom_map),1/res);
show(map);
[target1_xs,target1_ys,target2_xs,target2_ys,tracker]=set_target_tracker2; % assign path of two targets and tracker

% packing the target paths (should be same length)
targets_xs = [target1_xs ; target2_xs];
targets_ys = [target1_ys ; target2_ys];
N_target = 2; % only two targets will be considered

vis_cost_sets = {};

%% Get score maps of each target during a prediction horizon 
d_ref = 2;
for n = 1:N_target 
    target_xs = targets_xs(n,:);
    target_ys = targets_ys(n,:);
    
    vis_cost_set = []; % row : time / col : angle index 
    N_azim = 10;
    DT_set = [];
    for t = 1:length(target_xs) % -1 is due to some mistake in set_target_tracker function 
        target_position = [target_xs(t), target_ys(t) 0]';


        ray_len = norm(target_position(1:2) - tracker); % should decide this length 
        ray_len = d_ref;
        ray_lens = ray_len * ones(N_azim,1); 
        angles = linspace(0,2*pi,N_azim+1);
        angles(end) = [];

        cast_res = zeros(1,N_azim); % 1 for hit and 0 for free rays   
        collisionPts=map.rayIntersection(target_position,angles,ray_len); % the NaN elements are not hit, 
        collisionIdx=find(~isnan(collisionPts(:,1))); % collided index 
        cast_res(collisionIdx) = 1;

        DT = signed_distance_transform([cast_res cast_res cast_res]); % periodic distance transform         
        DT = DT(N_azim+1:2*N_azim);        
        DT_set = [DT_set ; DT ];    
        
       
        if sum(DT==inf) % in this case, no occlusion occurs
            vis_cost = zeros(1,N_azim);
            vis_cost_set(t,:) = vis_cost;
        else
            vis_cost = max(DT) - DT + 1;         
            vis_cost_set(t,:) = vis_cost;               
        end
        
        
    end
    % save for each agent 
    vis_cost_sets{n} = vis_cost_set;
    DT_sets{n} = DT_set;
    
end

%% Phase1 : feasible region convex division for LP problem 

% for now, the feasible region (search space) for solving discrete path is just rectangle
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

search_spec.x_dom = [xl,xu];
search_spec.y_dom = [yl yu];

% plot the problem 
show(map)
hold on 
plot(target1_xs,target1_ys,'r^-','LineWidth',2)
plot(target2_xs,target2_ys,'r^-','LineWidth',2)
plot(tracker(1),tracker(2),'ko')
patch([xl xu xu xl],[yl yl yu yu],'red','FaceAlpha',0.1)

color_set = {'g','b'};
visi_info_set = {};

% let's investigate inequality matrix for each time step 
for n = 1:N_target
    % target path 
    target_xs = targets_xs(n,:);
    target_ys = targets_ys(n,:);
    
    % A_sub, b_sub : inequality matrix of each sub division region (only available region)
    Nh = zeros(1,H); % we also invesigate number of available regions per each time step 
    angles = linspace(0,2*pi,N_azim+1);
    S = []; % flattened visibility score for easy computation in optimization     
    S_h = {}; % sorted way, this structure has same order  
    A_sub  = {};
    b_sub = {};
    N_vis_show = 3; % shows N largest vis score 
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
                if(DT_sets{n}(h,k)) % occlusion and collision rejection
                    [A,b] = get_ineq_matrix([target_xs(h) ; target_ys(h)],v1,v2);        
                    Nk = Nk +1;
                    A_sub{h}{Nk} = A; b_sub{h}{Nk}=b; % inequality constraint
                    S=[S vis_cost_sets{n}(h,k)]; % visbiility cost of the region 
                    S_h{h}{Nk} = vis_cost_sets{n}(h,k); % 
                end
            end          
        end

        if (sum(vis_cost_sets{n}(h,:))) % then, every bearing direction is just ok             
            N_vis_show = 3;
        else
            N_vis_show = N_azim;
        end
        
        [sorted_val, indices] = sort(vis_cost_sets{n}(h,:));
        goods = indices(1:N_vis_show);        
        for i = 1:N_vis_show
            good = goods(i);
            alpha_val = 1/sorted_val(i);
            if alpha_val == inf 
                alpha_val = 0.3;                
            end
            draw_circle_sector([target_xs(h) target_ys(h)],2*pi/N_azim * (good-1),2*pi/N_azim * (good),d_ref/2,color_set{n},alpha_val*0.3);        
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

% intersection region of the two pizza corresponding to each target

A_div = {};
b_div = {};
vis_cost_set = {};
Nk = 0; % number of valid regions 
ratio = 1; % importance ratio of vis2 to vis1
for h = 1:H    
    A_div{h} = {};
    b_div{h} = {};
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
                              
            A_intsec = [Ai ; Ai];
            b_intsec = [bi ; bj];
            
            [~,~,flag]=linprog([],[Ai ; Ai],[bi ; bj],[],[],[xl yl],[xu yu]);
            
                       
            if (flag ~= -2) % feasibility test pass
               % let's keep this region
               Nk = Nk + 1;
               A_div{h}{Nk} = A_intsec;
               b_div{h}{Nk} = b_intsec; 
               vis_cost_set{h}{Nk} =  vis_cost1 + ratio * vis_cost2; % let's assign the visibility cost to here                        
            end
            
        end        
    end
end


%% Plot faesible segment 
figure
for h =1 :H 
    subplot(2,H/2,h)
    for k = 1:length(vis_cost_set{h})
        
        alpha = 1/vis_cost_set{h}{k};        
        plot_feasible(A_div{h}{k},b_div{h}{k},[0 0],[xl yl]',[xu yu]', ...
            'backgroundcolor',[0.1 0.8 0.1],...
            'alpha',alpha);        
    end
    
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
MILP_path_plot1=plot(waypoints_x,waypoints_y,'ms-','LineWidth',2);
MILP_path_plot2=plot([tracker(1) waypoints_x(1)],[tracker(2) waypoints_y(1)],'ms-','LineWidth',2);



% legend(MILP_path_plot1,'MILP')

select_region_seq = zeros(1,H);
inspect_idx = 6*H -4 + 1;

% for h = 1:H
%     inspect = sol(inspect_idx : inspect_idx + Nh(h) -1);    
%     select_region_seq(h) = find(uint8(inspect)==1);
%     inspect_idx = inspect_idx + Nh(h);   
% end


%% Optional 

 figure;
for h = 1:H   
subplot(H/2,H/2,h)
fig_h=show(map);
axis off
hold on 
target_pos=plot(target_xs(h),target_ys(h),'r^','MarkerSize',10,'MarkerFaceColor','r');    
patch([xl xu xu xl],[yl yl yu yu],'red','FaceAlpha',0.1)

Nk = 0;
for k = 1:N_azim    
        % Bounding lines of the k th pizza segment !    
        draw_circle_sector([target_xs(h) target_ys(h)],2*pi/N_azim * (k-1),2*pi/N_azim * (k),10,'g',DT_set(h,k)/5); 
        text_theta = (2*pi/N_azim * (k-1) + 2*pi/N_azim * (k))/2;
        if (DT_set(h,k) )
            Nk = Nk + 1;
            text_str = sprintf('$z_{%d%d}$',h,Nk);
%             text(target_xs(h) + 3*cos(text_theta)  , target_ys(h) +3* sin(text_theta),text_str,'Interpreter','latex','FontSize',20)
        end
end

MILP_path_plot3=plot(waypoints_x,waypoints_y,'ms-','LineWidth',2);
plot(waypoints_x(h),waypoints_y(h),'ms','MarkerSize',15,'MarkerFaceColor','m');

text_str = sprintf('$x_{%d} , y_{%d} $',h,h);
text_str2 = sprintf('$x_{g,%d} , y_{g,%d} $',h,h);

text(waypoints_x(h) , waypoints_y(h) -0.5,text_str,'Interpreter','latex','FontSize',20)
text(target_xs(h) +0.3 , target_ys(h) ,text_str2,'Interpreter','latex','FontSize',20)


leg=legend([target_pos,MILP_path_plot3],{'target','wpts seq'},'Interpreter','latex','fontsize', 15);

title(sprintf('$ t_{%d}$',h),'Interpreter','latex','FontSize',20)
set(gca,'XLabel',[])
set(gca,'YLabel',[])

end

