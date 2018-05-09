classdef path_manager < handle 
    %% properties 
    properties 
        px; 
        py;
        obstacles;   % cell of obstacle    
    end
    %% methods 
    methods        
        function obj=path_manager(obstacles)
            obj.obstacles=obstacles;
            obj.px={};
            obj.py={};            
        end    
        
              
        %% utils 
        function vec=t_vec(~,n,t,n_diff)
            % this computes time vector 
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
            % in the boundary of box
            % (i,j) elem of map = lower_left + (dx/2 + (i-1) * dx ,dy/2+  (j-1) * dy )
            
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
           % this should be turned 
           map=(SEDT<3);               
                     
        end
        
        
        function waypoints=guidance_waypoint(path_manager,x0,xf,lower_left_corner,upper_right_corner,Nx,Ny,N_pnts)
            % this function calculate waypoints that derive the path to
            % avoid obstacle region
            % box should be larget than acutal BB including the x0, xf 
            % N_pnts : how many guidance point 
            % waypoints : N_pnts x 2
            inflated_map=path_manager.obs_inflation(lower_left_corner,upper_right_corner,Nx,Ny);            
            local_path=A_star(inflated_map,x0,xf,lower_left_corner,upper_right_corner);
            path_len=length(local_path);
            guidance_idx=floor(linspace(1,path_len,N_pnts+2));
            guidance_idx=guidance_idx(2:end-1);
            waypoints=local_path(guidance_idx,:);
                   
        end
        
        
        
        
        
        
            
        
        
            
            
            
       end
        
        
        
    
    
end