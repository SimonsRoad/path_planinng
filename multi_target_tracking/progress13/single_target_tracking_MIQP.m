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

%% Generation of optimal sequence : everything was converted into QP (refer lab note)


%%%
% Optimization variables 
% X = [x1 y1 x2 y2 ... xH yH  ||  z_1,1 z_1,2 ... z_1,N1 | ..| z_H,1 ... z_H,N_H 
%  (X+, X-), Ax, (Y+, Y-), Ay, D ]
% w_v: weight for visibility 
% w_d: weight for desired distance
%%%

w_v = 50;
w_d = 2; % weight for desired distance

N_var = 2*H + length(S) + 7*H;

%% Objective function 

% - 1st order
f = [reshape([target_xs ; target_ys],1,[]),  w_v*S zeros(1,7*H)]; 
% desired tracking distance term 
d_des_f_idx1 = 2*H + length(S) + 7:7:N_var;
f(d_des_f_idx1 ) = w_d;

% - 2nd order 
% Diagonal
Q = zeros(N_var);
Q(1:2,1:2) = eye(2); Q(2*H-1 : 2*H, 2*H-1:2*H) = eye(2);
Q(3:2*H-2,3:2*H-2) = 2*eye(2*(H-2));

% Super diagonal 
for h = 1:H-1
    Q(2*(h-1)+1:2*h,2*h+1:2*(h+1)) = -eye(2);
    Q(2*h+1:2*(h+1),2*(h-1)+1:2*h) = -eye(2);    
end

Q = 2*Q;
%% Desired tracking distance equality and inequality 

d_des = 3;

Aineq1 = zeros(14*H,N_var); bineq1 = zeros(14*H,1);
Aeq1 = zeros(2*H,N_var); beq1 = zeros(2*H,1);

until_this=2*H + length(S);
for h = 1 : H 
    % var : until this ++ X+, X-, Y+, Y-, Ax, Ay, D  ++ X, Y  
    xc = target_xs(h); yc = target_ys(h);
    Lx = xl - xc; Ux = xu - xc;
    Ly = yl - yc; Uy = yu - yc;
    
    Aineq1_blk = [
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

  bineq1_blk = [0 0 -max(0,-Ux) max(0,-Lx) 0 0 -max(0,-Uy) max(0,-Ly) -d_des d_des -((abs(Lx)+Lx)/(2*abs(Lx))) ((abs(Ux)+Ux)/(2*abs(Ux))) -((abs(Ly)+Ly)/(2*abs(Ly))) ((abs(Uy)+Uy)/(2*abs(Uy)))]';
  
  Aineq1( 14*(h-1)+1 : 14*h , until_this + 7*(h-1) +1 : until_this + 7*(h) ) = Aineq1_blk;
  bineq1( 14*(h-1)+1 : 14*h ) = bineq1_blk;
    
  Aeq1(2*(h-1)+1 : 2*h, 2*(h-1)+1 : 2*h) = eye(2); 
  Aeq1(2*(h-1)+1 : 2*h, until_this + 7*(h-1)+1 :until_this + 7*(h-1) + 4) = [-1 1 0 0; 0 0 -1 1]; 
  beq1(2*(h-1)+1 : 2*h) = [target_xs(h) ; target_ys(h)]; 
 
end


%% Subdivision 
% region selection inequality 
M = 1e+2; % some large number (big enough but not too be big)

dim = 2; % currently 2D
Aineq2 = zeros(dim * sum(Nh),N_var);
bineq2 = M*ones(dim * sum(Nh),1);
insert_blk_row = 1;

for h = 1:H 
    for k = 1: Nh(h)
        Aineq2(2*(insert_blk_row-1)+1:2*(insert_blk_row),2*(h-1)+1:2*(h)) = A_sub{h}{k};
        Aineq2(2*(insert_blk_row-1)+1:2*(insert_blk_row), 6*H-4 + (insert_blk_row)) = -b_sub{h}{k} + M; 
        insert_blk_row = insert_blk_row + 1;
    end   
end

% region selection equality
Aeq2 = zeros(H,N_var);
insert_idx = 2*H  + 1;
    
for h = 1:H
    Aeq2(h,insert_idx:insert_idx+Nh(h)-1) = ones(1,Nh(h));
    insert_idx = insert_idx+Nh(h)  ;
end
beq2 = ones(H,1);
   

%% Optimization 

xtype = [repmat('C',1,2*H) repmat('B',1,length(S)) repmat('C',1,7*H) ];
int_idx1 = 2*H + length(S) + 5:7:N_var;
int_idx2 = 2*H + length(S) + 6:7:N_var;
xtype([int_idx1 int_idx2]) = 'I';

A = [Aineq1 ; Aineq2];
b = [bineq1; bineq2];

Aeq = [Aeq1 ; Aeq2];
beq =[beq1 ; beq2];
lbs = -inf * ones(N_var,1); lbs(1:2:2*H) = xl;  lbs(2:2:2*H) = yl;
ubs = inf * ones(N_var,1); ubs(1:2:2*H) = xu; ubs(2:2:2*H) = yu;
Opt = opti('qp',Q,f,'ineq',A,b,'eq',Aeq,beq,'lb',lbs,'ub',ubs,'xtype',xtype);
[x,fval,exitflag,info] = solve(Opt);





















