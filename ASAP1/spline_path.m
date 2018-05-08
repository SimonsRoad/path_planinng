classdef spline_path < handle 
    properties
        n_poly; % order of poly of each segment 
        px; % cell of  poly coeffi of x
        py; % cell of poly coeffi of y
        waypoints; % cell of positions (includes initial state)
        n_seg; % # of seg of spline 
        t_horizon % actual time horizion from start to goal     
        w_d; % effort to cross the waypoints 
        x0;
        xdot0;
    end
    
    methods
        function obj=spline_path(n_poly,x0,xdot0,waypoints,w_v)
            obj.n_poly=n_poly;
            obj.px={};
            obj.py={};
            obj.x0=x0;
            obj.xdot0=xdot0;
            obj.waypoints=waypoints;
            obj.n_seg=length(waypoints)-1;            
            obj.w_d=w_v;
        end
               
        
        function generate_spline(spline_path)
            % we find the path coeffi via QP  p'Qp+Hp
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
                if seg==1
                    % if this is first segments, then constraint is imposed
                    
                    % desired goal point of this segments 
                    xd=spline_path.waypoints{seg}(1); yd=spline_path.waypoints{seg}(2);                                        
                    Q_xd = spline_path.t_vec(0,1)*spline_path.t_vec(0,1)';   Q_yd=Q_xd;
                    H_xd=-2*xd*(spline_path.t_vec(1,0))'; H_yd=-2*yd*(spline_path.t_vec(1,0))';
                    
                    % weight of goal following 
                    w=spline_path.w_d;
                    Qx=Q_j+w*Q_xd; Hx=w*H_xd;
                    Qy=Q_j+w*Q_yd; Hx=w*H_xd;

                    % optimization                     
                    A=[spline_path.t_vec(0,0)' ; spline_path.t_vec(0,1)' ];
                    bx=[spline_path.x0(1) ; spline_path.xdot0(1)]; by=[spline_path.x0(2) ; spline_path.xdot0(2)];
                    spline_path.px{seg}  = quadprog(2*Qx,Hx,[],[],A,bx);
                    spline_path.py{seg}  = quadprog(2*Qy,Hy,[],[],A,by);                                        
                else
                    
                end % if else end 
             
            end
            
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
    
    
    end

end