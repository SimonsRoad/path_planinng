%% MINIMUM JERK TARJECTORY GENERATION (SOFT VERSION : WAYPOINTS SOFT )
% INPUT = [ts,xs] (waypoint and corresponding time)

function sol=min_jerk_soft(ts,xs,ys,zs)
    
    n_seg=length(ts)-1;
    poly_order=5; % for jerk, order 5 is enough 
   
    % QP FORM : p'Qp+Hp
    
    %% construct collective jerk matrix 
    Q_jerk=zeros((poly_order+1)*n_seg);
   
    for n=1:n_seg
        % time scaling constant
        dn=ts(n+1)-ts(n);
        Dn=zeros(poly_order+1);
        for i=1:poly_order+1
            Dn(i,i)=dn^(i-1);
        end
        Q_jerk(poly_order*(n-1)+1:poly_order*n,poly_order*(n-1)+1:poly_order*n)=Dn*integral_jerk_squared(poly_order)*Dn/dn^5;
    end
    
    %% soft waypoint cost 
    
    Q_wpnt=zeros((poly_order+1)*n_seg);
    H_wpnt_x=zeros((poly_order+1)*n_seg,1);
    H_wpnt_y=H_wpnt_x;
    H_wpnt_z=H_wpnt_x;
    
    for n=1:n_seg
        
        % time scaling constant
        dn=ts(n+1)-ts(n);
        Dn=zeros(poly_order+1);
        for i=1:poly_order+1
            Dn(i,i)=dn^(i-1);
        end
        
        Q_wpnt(poly_order*(n-1)+1:poly_order*n,poly_order*(n-1)+1:poly_order*n)=Dn*t_vec(poly_order,1,0)*t_vec(poly_order,1,0)'*Dn;
        H_wpnt_x(poly_order*(n-1)+1:poly_order*n)=-2*xs(n+1)*t_vec(poly_order,1,0)'*Dn;
        H_wpnt_y(poly_order*(n-1)+1:poly_order*n)=-2*ys(n+1)*t_vec(poly_order,1,0)'*Dn;
        H_wpnt_z(poly_order*(n-1)+1:poly_order*n)=-2*zs(n+1)*t_vec(poly_order,1,0)'*Dn;
       
    end
    
    
    
    
   

end
