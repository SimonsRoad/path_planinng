classdef spline_path < handle 
    properties
        n_poly; % order of poly of each segment 
        px; % cell of  poly coeffi of x
        py; % cell of poly coeffi of y
        waypoints; % cell of positions (includes initial state)
        n_seg; % # of seg of spline 
        t_horizon % actual time horizion from start to goal     
        w_d; % effort to cross the waypoints 
        w_o; % obstacle avoidance 
        x0; % initial state of the traj 
        xdot0; 
        obstacle; % only particle obstacles are considered 
    end
    
    methods
        %% generating the path 
        % this function generate one chunk of  polynomial function that
        % tries to pass the waypoints 
        function generate_one_poly(spline_path)
            % poly order 
            n=spline_path.n_poly;
            % jerk minimization 
            Q_j=zeros(n+1);            
            for i=4:n+1
                for j=4:n+1
                    if i==4 && j==4
                        Q_j(i,j)=(i-1)*(i-2)*(i-3)*(j-1)*(j-2)*(j-3);
                    else
                        Q_j(i,j)=(i-1)*(i-2)*(i-3)*(j-1)*(j-2)*(j-3)/(i+j-8);
                    end
                end
            end
                        
            % accel minimization 
            Q_a=zeros(n+1);            
            for i=3:n+1
                for j=3:n+1
                    if i==3 && j==3
                        Q_a(i,j)=(i-1)*(i-2)*(j-1)*(j-2);
                    else
                        Q_a(i,j)=(i-1)*(i-2)*(j-1)*(j-2)/(i+j-6);
                    end
                end
            end
            
        tm=linspace(0,1,spline_path.n_seg+1); % this is the time passing through waypoints 
        Qx=Q_a; Qy=Qx; Hx=zeros(1,n+1); Hy=Hx;
        w=spline_path.w_d;
       for seg=1:spline_path.n_seg            
            xd=spline_path.waypoints{seg}(1); yd=spline_path.waypoints{seg}(2);
            t=tm(seg+1);
            Q_xd = spline_path.t_vec(t,0)*spline_path.t_vec(t,0)';   Q_yd=Q_xd;
            H_xd=-2*xd*(spline_path.t_vec(t,0))'; H_yd=-2*yd*(spline_path.t_vec(t,0))';
            Qx=Qx+w*Q_xd; Qy=Qy+w*Q_yd;
            Hx=Hx+w*H_xd; Hy=Hy+w*H_yd;            
        end
            
        A=[spline_path.t_vec(0,0)' ; spline_path.t_vec(0,1)' ];
        bx=[spline_path.x0(1) ; spline_path.xdot0(1)]; by=[spline_path.x0(2) ; spline_path.xdot0(2)];
        spline_path.px{1}  = quadprog(2*Qx,Hx,[],[],A,bx);
        spline_path.py{1}  = quadprog(2*Qy,Hy,[],[],A,by);        
            
        end
        
        % this function generate piecewise polynomial function 
        function generate_spline(spline_path)
            % we find the path coeffi via QP  p'Qp+Hp
            n=spline_path.n_poly;
            Q_j=zeros(n+1);            
            for i=4:n+1
                for j=4:n+1
                    if i==4 && j==4
                        Q_j(i,j)=(i-1)*(i-2)*(i-3)*(j-1)*(j-2)*(j-3);
                    else
                        Q_j(i,j)=(i-1)*(i-2)*(i-3)*(j-1)*(j-2)*(j-3)/(i+j-8);
                    end
                end
            end
            
            % spline generation 
            for seg=1:spline_path.n_seg
                                               
                if seg ==1 
                    x_init=spline_path.x0; xdot_init=spline_path.xdot0;
                else
                    x_init=[(spline_path.px{seg-1})'*spline_path.t_vec(1,0) (spline_path.py{seg-1})'*spline_path.t_vec(1,0)];
                    xdot_init=[(spline_path.px{seg-1})'*spline_path.t_vec(1,1) (spline_path.py{seg-1})'*spline_path.t_vec(1,1)];
                end
                % desired goal point of this segments 
                xd=spline_path.waypoints{seg}(1); yd=spline_path.waypoints{seg}(2);                                        

                % consider the obstacles
                % if obstacles is within this boundary,  potential field 
                boundary_rad=norm(x_init-[xd yd])/2;
                boundary_center=(x_init+[xd yd])/2;
                obstacle_seg={};
                insert_idx=1;
                for idx=1:length(spline_path.obstacle)
                    obs_pos=spline_path.obstacle{idx};
                    if norm(obs_pos-boundary_center)< boundary_rad                        
                        obstacle_seg{insert_idx}=obs_pos;
                        insert_idx=insert_idx+1;
                    end                    
                end
                
                % points to stay away from obstacle                    
                N_sample=4;
                t_obs_sample=linspace(0,1,N_sample+2);
                t_obs_sample=t_obs_sample(2:end-1);
                                               
                % objective functions 
                Q_xd = spline_path.t_vec(1,0)*spline_path.t_vec(1,0)';   Q_yd=Q_xd;
                H_xd=-2*xd*(spline_path.t_vec(1,0))'; H_yd=-2*yd*(spline_path.t_vec(1,0))';
                
                for i=1:length(obstacle_seg)
                    obs_pose=obstacle_seg{i};
                    for j=1:N_sample
                        Q_xo= spline_path.t_vec(t_obs_sample(j),0)*spline_path.t_vec(t_obs_sample(j),0)';
                        Q_yo=Q_xo;
                        H_xo=-2 * obs_pose(1) * spline_path.t_vec(t_obs_sample(j),0)'; 
                        H_yo=-2 * obs_pose(2) * spline_path.t_vec(t_obs_sample(j),0)';                        
                    end
                end

                w_d=spline_path.w_d;
                w_o=spline_path.w_o;
                
%                 Qx=Q_j+w_d*Q_xd-w_o*Q_xo; Hx=w_d*H_xd-w_o*H_xo;                            
%                 Qy=Q_j+w_d*Q_yd-w_o*Q_yo; Hy=w_d*H_yd-w_o*H_yo;
                 
                
                Qx=Q_j-w_o*Q_xo; Hx=-w_o*H_xo;
                Qy=Q_j-w_o*Q_yo; Hy=-w_o*H_yo;

                % optimization                     
%                 A=[spline_path.t_vec(0,0)' ; spline_path.t_vec(0,1)' ];
%                 bx=[x_init(1) ; xdot_init(1)]; by=[x_init(2) ; xdot_init(2)];
%              
               % waypoint equality constraints 
                Aeq=[spline_path.t_vec(0,0)' ; spline_path.t_vec(0,1)' ; spline_path.t_vec(1,0)' ];
                beqx=[x_init(1) ; xdot_init(1) ; xd]; beqy=[x_init(2) ; xdot_init(2) ; yd];
                
               % corridor ineq constraint (within the inflated box)
               left_lower_corner= [min(x_init(1), xd)-1 min(x_init(2),yd)-1];                
               right_upper_corner= [max(x_init(1), xd)+1 max(x_init(2),yd)+1];
               
               
                
                spline_path.px{seg}  = quadprog(2*Qx,Hx,[],[],Aeq,beqx);
                spline_path.py{seg}  = quadprog(2*Qy,Hy,[],[],Aeq,beqy);                                        
                
            end
            
        end
        
        %% miscellaneous
       function obj=spline_path(n_poly,x0,xdot0,waypoints,w_d,w_o,obstacle)
            obj.n_poly=n_poly;
            obj.px={};
            obj.py={};
            obj.x0=x0;
            obj.xdot0=xdot0;
            obj.waypoints=waypoints;
            obj.n_seg=length(waypoints);            
            obj.w_d=w_d;
            obj.w_o=w_o;
            obj.obstacle=obstacle;
        end
        
                
       function vec=t_vec(spline_path,t,n_diff)
            % this computes time vector 
            vec=zeros(spline_path.n_poly+1,1);
            switch n_diff
                case 0
                    for i=1:spline_path.n_poly+1
                        vec(i)=t^(i-1);
                    end            
                case 1
                    for i=2:spline_path.n_poly+1
                        vec(i)=(i-1)*t^(i-2);
                    end
                    
                case 2
                    for i=3:spline_path.n_poly+1
                        vec(i)=(i-1)*(i-2)*t^(i-3);
                    end
                       
            end        
        end
    
        

        function path_plot(spline_path)
            % this function plot the path
            hold on 
            % obstalce 
             for idx=1:length(spline_path.obstacle)
                    obs_pos=spline_path.obstacle{idx};
                    plot(obs_pos(1),obs_pos(2),'ko','MarkerSize',12)
             end
                
            
            % first, we draw the wapoints given to the class             
            plot(spline_path.x0(1),spline_path.x0(2),'r*','LineWidth',3);
            
            for i=1:length(spline_path.waypoints)
                wp=spline_path.waypoints{i};
                plot(wp(1),wp(2),'r*','LineWidth',3)
            end
            
            % for each segment, we plot the path 
            for seg=1:length(spline_path.px)
                px_seg=spline_path.px{seg};
                py_seg=spline_path.py{seg};                
                t=linspace(0,1,20);
                x=zeros(length(t),1);
                y=x;
                for i=1:length(t)
                    t_=t(i);
                    x(i)=px_seg'*spline_path.t_vec(t_,0);    
                    y(i)=py_seg'*spline_path.t_vec(t_,0);
                end                
                plot(x,y,'g-')                               
            end                        
        end
    
    
    end

end