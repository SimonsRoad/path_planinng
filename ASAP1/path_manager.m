classdef path_manager < handle 
    %% properties 
    properties 
        px; 
        py;
        obstacles;   % cell of obstacle 
        Q_j; % jerk matrix 
        Q_a; % acceleration matrix 
        path_param; % params ={inflation rate, box extension }
    end
    %% methods 
    methods        
        function obj=path_manager(obstacles,path_param)            
            obj.obstacles=obstacles;
            obj.px={};
            obj.py={};        
            obj.path_param=path_param;
        end    
        
        function cost_mat_gen(path_manager,n)             % COMPUTE JERK OR ACCEL MATRIX 
            Q_j=zeros(n+1);            
            for i=4:n+1
                for j=4:n+1
                    if i==4 && j==4
                        Q_j(i,j)=(i-1)*(i-2)*(i-3)*(j-1)*(j-2)*(j-3);
                    else
                        Q_j(i,j)=(i-1)*(i-2)*(i-3)*(j-1)*(j-2)*(j-3)/(i+j-7);
                    end
                end
            end

            Q_a=zeros(n+1);            
            for i=3:n+1
                for j=3:n+1
                    if i==3 && j==3
                        Q_a(i,j)=(i-1)*(i-2)*(j-1)*(j-2);
                    else
                        Q_a(i,j)=(i-1)*(i-2)*(j-1)*(j-2)/(i+j-5);
                    end
                end
            end            
            path_manager.Q_a=Q_a;
            path_manager.Q_j=Q_j;                  
                   
        end
        
        
        function obs_update(path_manager,obs)
            N_obs_cur=length(path_manager.obstacles);
            path_manager.obstacles{N_obs_cur+1}=obs;            
        end
        
        %% path generation  ( poly_traj_gen needed ! ) 
        function [xs,vs,xs_seg,vs_seg]=traj_gen(path_manager,x0,xdot0,waypoints,N_iter)            
            % this function generate trajectory. the polynomial coeff can
            % be found in path_manager (px py) in form of cell 
            % x0, xdot0 ( 1 x 2) / waypoints ( N x 2)
            
           % OUTPUTS
           %%%%%%%%%%%%%%%%%%%%%
           % xs,vs : stats along the whole trajectory 
           % xs_seg, v_seg : states of each segment
           %%%%%%%%%%%%%%%%%%%%%
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % TUNEING : polynomial order (n) / max_iter for optimization /
            % bounding value of vset 
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            %% PARSING
            N_waypoints=size(waypoints,1);
            waypoint={}; % waypoint is cell (~= waypoints)
            for i=1:N_waypoints
                waypoint{i}=waypoints(i,:);
            end
            
            
            %%  GUIDANCE WAYPOINTS 
            guidance_waypoint={};             
            % Too big extension causes unnecessary guindace points
            Nx_grid=25;
            Ny_grid=25;
            N_guide=path_manager.path_param.N_guide;

            for i=1:N_waypoints
                if i==1
                    % For each segment 
                    x_init=x0; xd=waypoint{i}(1); yd=waypoint{i}(2);                   
                else                                    
                    x_init=waypoint{i-1}; xd=waypoint{i}(1); yd=waypoint{i}(2);
                end
                    
                    % Let's consider circle 
                    center=(x_init+[xd yd])/2;
                    rad=norm([xd yd]-x_init)/2;
%                     left_lower_corner= [min(xd,x_init(1)) min(yd,x_init(2))]-box_extension*[1 1];
%                     right_upper_corner=  [max(xd,x_init(1)) max(yd,x_init(2))]+box_extension*[1 1];    
                    left_lower_corner=center-(rad+path_manager.path_param.box_extension)*[1 1];
                    right_upper_corner=center+(rad+path_manager.path_param.box_extension)*[1 1];
                    % Generate path using A* for guidance points
                    guidance_pnts=path_manager.guidance_waypoint(x_init,[xd yd],left_lower_corner,right_upper_corner,Nx_grid,Ny_grid,N_guide);
                    guidance_waypoint{i}=guidance_pnts;                    
            end
            %% OPTIMIZATION 
            n=7; % poly order 
            vset=(waypoint{1}-x0)';
            for i=2:length(waypoint)
                vset=[vset ; waypoint{i}(1)-waypoint{i-1}(1) ; waypoint{i}(2)-waypoint{i-1}(2) ];
            end

            % function handle
            path_manager.cost_mat_gen(n); % This computes jerk and acceleration matrix 
            sum_cost=@(v) poly_traj_gen(v,n,x0,xdot0,waypoint,guidance_waypoint,path_manager);

            %%%%%% two-stage optimization %%%%%%%%%%%%%%%%%
            % inner : fast QP 
            % outer : non linear optimization for less cost using
            % connecting velocity 
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % bound of velocity
            upper_limit=1.5;
            lower_limit=-1.5;
            len=length(vset);

            options = optimoptions('fmincon','Algorithm','SQP','MaxFunctionEvaluations',N_iter);
            fmincon(sum_cost,vset,[],[],[],[],lower_limit*ones(len,1),upper_limit*ones(len,1),[],options);  
            %% PATH POINTS SAVE 
            xs=[]; xs_seg={};
            vs=[]; vs_seg={};
            % for each segment, we construct the path 
            for seg=1:length(path_manager.px)
                px_seg=path_manager.px{seg};
                py_seg=path_manager.py{seg};    
                n_poly=length(px_seg)-1;
                t=linspace(0,1,20);
                x=zeros(length(t),1);
                y=x; vx=y; vy=vx;
                for i=1:length(t)
                    t_=t(i);
                    x(i)=px_seg'*path_manager.t_vec(n_poly,t_,0);    
                    y(i)=py_seg'*path_manager.t_vec(n_poly,t_,0);
                    vx(i)=px_seg'*path_manager.t_vec(n_poly,t_,1);
                    vy(i)=py_seg'*path_manager.t_vec(n_poly,t_,1);                                        
                end                                                                     
                xs=[xs ; [x y]];
                vs=[vs ; [vx vy]];
                xs_seg{seg}=[x y];
                vs_seg{seg}=[vx vy];
            end      
            

        end
        
        
        %% utils 
        % time vector for fast computation of polynomial 
        function vec=t_vec(~,n,t,n_diff)
            % this computes time vector.
            % n: order of poly
            % t: eval point 
            % n_diff : diff order 
            
            vec=zeros(n+1,1);
            switch n_diff
                case 0
                    for i=1:n+1
                        vec(i)=t^(i-1);
                    end            
                case 1
                    for i=2:n+1
                        vec(i)=(i-1)*t^(i-2);
                    end
                    
                case 2
                    for i=3:n+1
                        vec(i)=(i-1)*(i-2)*t^(i-3);
                    end                       
            end        
        end
        
        
        function path_plot(path_manager)               
            % for each segment, we plot the path 
            for seg=1:length(path_manager.px)
                px_seg=path_manager.px{seg};
                py_seg=path_manager.py{seg};    
                n_poly=length(px_seg)-1;
                t=linspace(0,1,20);
                x=zeros(length(t),1);
                y=x;
                for i=1:length(t)
                    t_=t(i);
                    x(i)=px_seg'*path_manager.t_vec(n_poly,t_,0);    
                    y(i)=py_seg'*path_manager.t_vec(n_poly,t_,0);
                end                
                plot(x,y,'gs-','LineWidth',2);                               
            end                  
            
        end
        
        
        function mapplot(path_manager)
            % this funnction plot the current problem settings
            hold on 
                       
            for i=1:length(path_manager.obstacles)
                obs=path_manager.obstacles{i};
                obs.plot
            end
            
            hold off
        end
        
        function map=obs_inflation(path_manager,lower_left_corner,upper_right_corner,Nx,Ny)
            % corner : 1 x 2
            % for A* local planner , computes inflated binary occupancy map
            % in the boundary of box defined by two corner 
            % (i,j) elem of map = lower_left + (dx/2 + (i-1) * dx ,dy/2+  (j-1) * dy )
            % Nx, Ny normally 15
            
            map=zeros(Nx,Ny);
            dx=(upper_right_corner(1)-lower_left_corner(1))/Nx;            
            dy=(upper_right_corner(2)-lower_left_corner(2))/Nx;
                        
            for ix=1:Nx
                for iy=1:Ny
                    cur_pos=lower_left_corner+ [dx/2+(ix-1)*dx dy/2+(iy-1)*dy ];
                    
                    % obstacle check 
                    for obs_idx=1:length(path_manager.obstacles) % for obstacle list
                        this_obs=path_manager.obstacles{obs_idx};
                        if this_obs.isobs(cur_pos')
                            map(ix,iy)=1;
                            break
                        end                        
                    end  % for all obstacle
                    
                end                               
            end 
                            
           SEDT=signed_distance_transform(map); 
           % threshold should be turned 
           map=(SEDT<(Nx+Ny)/2*path_manager.path_param.inflate_rate);               
                     
        end
                
        function waypoints=guidance_waypoint(path_manager,x0,xf,lower_left_corner,upper_right_corner,Nx,Ny,N_pnts)
            % this function calculate waypoints that derive the path to
            % avoid obstacle region
            % box should be larget than acutal BB including the x0, xf 
            % N_pnts : how many guidance point 
            % waypoints : N_pnts x 2
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % TUNEING : bedding size             
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                        
            waypoints=[];            
            inflated_map=path_manager.obs_inflation(lower_left_corner,upper_right_corner,Nx,Ny);            
            if sum(sum(inflated_map))==0
                return % no obstacle found!
            end
            local_path=A_star(inflated_map,x0,xf,lower_left_corner,upper_right_corner);
            path_len=length(local_path);
            guidance_idx=floor(linspace(1,path_len,N_pnts+2));
            guidance_idx=guidance_idx(2:end-1);
            waypoints=local_path(guidance_idx,:);                   
        end           
          
        function waypoints = comparison_algorithm(path_manager,x0,target_prediction) 
            % this function is for generating a tracking path which is not
            % my algorithm. The x0 is current tracker positoin.
            % we first generate initial tracking path by traslation
            
            cur_target_pos = target_prediction(1,:); 
            
            N_points = length(target_prediction); % N_pred + 1
            
            x0 = reshape(x0,1,2);
            trans = x0 - cur_target_pos;
            % this is would be final position of the tracker following this
            % simple algorithm 
            xf_ = target_prediction(end,:) + trans ;
            % let't inspect whether this point feasible 
            xf_try = xf_;
            is_feasible = true ;
            for obs_idx=1:length(path_manager.obstacles) % for obstacle list
                   this_obs=path_manager.obstacles{obs_idx};
                   if this_obs.isobs(xf_')
                            is_feasible = false;
                         break
                    end      
            end
            % if this point was infeasible, purturb little bit in a circle
            % boundary r_max 
             r_max = 5; % hope feasible point is discovered in this boundary 
             
             try_num = 10000000;
             tried_num = 0;
             
            while (~is_feasible) 
                 r_sample = r_max * rand;
                 theta_sample = 2*pi * rand;
                    
                 xf_try = xf_ + [r_sample * cos(theta_sample) r_sample * sin(theta_sample)];
                 
                 % try feasiblity
                is_feasible = true ;
                for obs_idx=1:length(path_manager.obstacles) % for obstacle list
                       this_obs=path_manager.obstacles{obs_idx};
                       if this_obs.isobs(xf_try')
                                is_feasible = false;
                             break
                       end      
                end
               
                tried_num = tried_num + 1;
            end
            
             if (is_feasible)
                xf = xf_try; 
                % okay proceed with this final position. 
                % Too big extension causes unnecessary guindace points
                Nx_grid=25;
                Ny_grid=25;
                
                left_lower_corner = [min(x0(1),xf(1)) min(x0(2),xf(2))]-[1 1];
                right_upper_corner = [max(x0(1),xf(1)) max(x0(2),xf(2))]+[1 1];
                waypoints = path_manager.guidance_waypoint(x0,xf,left_lower_corner,right_upper_corner,Nx_grid,Ny_grid,N_points);
             else
                 fprintf("feasible final point was not found, try inceasing the search boundary");
             end        
            
        end
        
        
       end
          
end