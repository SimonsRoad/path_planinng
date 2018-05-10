function sum_jerk=poly_traj_gen(v_set,n_poly,x0,x0dot,waypoints,path_manager)
    % this function computes minimum jerk trajectory given waypoints and
    % corresponding velocities (v_set)
    % inputs %
    % v_set : flattened velocity vector of each waypoint (2N_seg x 1)
    % n_poly: polynomial order of each spline 
    % x0/x0dot : initial position / vel (2 x 1)
    % waypoints : cell of waypoints ()
    % output %
    % sum of jerk along the whole spline curves     
    
    % we perform 2-stage optimization
    
  %% parsing     
    n=n_poly;
    n_seg=length(waypoints);
    v_set=reshape(v_set,2,[])';
       
    %% jerk matrix 
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
    
    Q=Q_a;
    sum_jerk=0;
    
    
    
    for seg=1:n_seg
        % initial condition 
        if seg==1
            x_init=x0; xdot_init=x0dot;            
        else
            x_init=waypoints{seg-1}; xdot_init=v_set(seg-1,:);
        end
        
        xd=waypoints{seg}(1); yd=waypoints{seg}(2);                                        
        xdotd=v_set(seg,1); ydotd=v_set(seg,2);
        
        %% guidance waypoint generation inside the segment 
        % box considered for obstacle avoidance 
        left_lower_corner= [min(xd,x_init(1)) min(yd,x_init(2))]-[0.3 0.3];
        right_upper_corner=  [max(xd,x_init(1)) max(yd,x_init(2))]+[0.3 0.3];
                       
        
        % generate guidance points 
        Nx_grid=15;
        Ny_grid=15;
        N_guide=1;
        guidance_pnts=path_manager.guidance_waypoint(x_init,[xd yd],left_lower_corner,right_upper_corner,Nx_grid,Ny_grid,N_guide);
        
        if ~isempty(guidance_pnts)
        
        
        t_guide=linspace(0,1,N_guide+2);
        t_guide=t_guide(2:end-1);
        
        for j=1:N_guide
            Qgx= path_manager.t_vec(n,t_guide(j),0)*path_manager.t_vec(n,t_guide(j),0)';
            Qgy=Qgx;
            Hgx=-2 * guidance_pnts(j,1) * path_manager.t_vec(n,t_guide(j),0)'; 
            Hgy=-2 * guidance_pnts(j,2) * path_manager.t_vec(n,t_guide(j),0)';                        
        end        
        w=1e+3;                
        
        Qgx=Qgx*w; Qgy=Qgy*w; Hgx=Hgx*w; Hgy=Hgy*w;
        
        else
            
        Qgx=zeros(n+1); Qgy=Qgx; Hgx=zeros(1,n+1); Hgy=Hgx;
            
        end
        %% optimization 
        
        % waypoint equality constraints 
        Aeq=[path_manager.t_vec(n,0,0)' ; path_manager.t_vec(n,0,1)' ; path_manager.t_vec(n,1,0)' ; path_manager.t_vec(n,1,1)' ];
        beqx=[x_init(1) ; xdot_init(1) ; xd ; xdotd]; beqy=[x_init(2) ; xdot_init(2) ; yd ; ydotd];
        
                
        options = optimoptions('quadprog','Display','off');
        [path_manager.px{seg},seg_jerk_x]  = quadprog(2*Q+2*Qgx,Hgx,[],[],Aeq,beqx,[],[],[],options);
        [path_manager.py{seg},seg_jerk_y]  = quadprog(2*Q+2*Qgy,Hgy,[],[],Aeq,beqy,[],[],[],options);     
        
        sum_jerk=sum_jerk+seg_jerk_x+seg_jerk_y;            
    end
    

   
end
