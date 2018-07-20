%% MINIMUM JERK TARJECTORY GENERATION (SOFT VERSION : WAYPOINTS SOFT )
% INPUT = [ts,xs] (waypoint and corresponding time)

function sol=min_jerk_soft(ts,xs,ys,zs,xdot0,ydot0,zdot0)
    
    n_seg=length(ts)-1;
    poly_order=5; % for jerk, order 5 is enough 
   
    % QP FORM : p'Qp+Hp
    
    %% construct collective jerk matrix 
    
    n_var_total=(poly_order+1)*n_seg; % total number of optimization variables 
    Q_jerk=zeros((poly_order+1)*n_seg);
   
    for n=1:n_seg
        % time scaling constant
        
        Dn=time_scailing_mat(ts(n),ts(n+1),poly_order);
        
        Q_jerk((poly_order+1)*(n-1)+1:(poly_order+1)*n,(poly_order+1)*(n-1)+1:(poly_order+1)*n)=Dn*integral_jerk_squared(poly_order)*Dn/dn^5;
    end
    
    %% soft waypoint cost 
    
    Q_wpnt=zeros((poly_order+1)*n_seg);
    H_wpnt_x=zeros((poly_order+1)*n_seg,1);
    H_wpnt_y=H_wpnt_x;
    H_wpnt_z=H_wpnt_x;
    
    for n=1:n_seg
        
        % time scaling constant
        Dn=time_scailing_mat(ts(n),ts(n+1),poly_order);    
        insert_range=(poly_order+1)*(n-1)+1:(poly_order+1)*n;
        
        
        Q_wpnt(insert_range,insert_range)=Dn*t_vec(poly_order,1,0)*t_vec(poly_order,1,0)'*Dn;
        H_wpnt_x(insert_range)=-2*xs(n+1)*t_vec(poly_order,1,0)'*Dn;
        H_wpnt_y(insert_range)=-2*ys(n+1)*t_vec(poly_order,1,0)'*Dn;
        H_wpnt_z(insert_range)=-2*zs(n+1)*t_vec(poly_order,1,0)'*Dn;
       
    end
    
    %% equality constratint 
    
    Aeq=zeros(2,n_var_total);  beq_x  = zeros(2,1); beq_y = zeros(2,1); beq_z  = zeros(2,1);
    % initial constraint 
    % x0
    Aeq(1,1)=1; beq_x(1)= xs(1); beq_y(1)=ys(1); beq_z(1)=zs(1);
    % v0
    Aeq(2,2)=1; beq_x(2)=xdot0; beq_y(2)=ydot0; beq_z(2)=zdot0;
       
    % 0st order continuity 
    insert_range1=1:poly_order+1; insert_range2=insert_range1+poly_order+1;
    
    for n=1:n_seg-1
        Dn=time_scailing_mat(ts(n),ts(n+1),poly_order); % Dn
        Dn_1=time_scailing_mat(ts(n),ts(n+1),poly_order);%Dn+1

        Aeq_temp=zeros(1,n_var_total); 
        Aeq_temp(insert_range1) =  t_vec(poly_order,1,0)'*Dn;
        Aeq_temp(insert_range2) =  t_vec(poly_order,0,0)'*Dn_1; 
        % add equality constraint 
        Aeq=[Aeq ; Aeq_temp]; beq_x=[beq_x ; 0]; beq_y=[beq_y ; 0]; beq_z=[beq_z ; 0];
        insert_range1=insert_range2; insert_range2=insert_range1+poly_order+1;
    end
   
    % 1st order continuity 
   insert_range1=1:poly_order+1; insert_range2=insert_range1+poly_order+1;
    
    for n=1:n_seg-1
        
        dn = ts(n+1)-ts(n);
        dn_1=ts(n+2)-ts(n+1);
        
        Dn=time_scailing_mat(ts(n),ts(n+1),poly_order); % Dn
        Dn_1=time_scailing_mat(ts(n),ts(n+1),poly_order);%Dn+1

        Aeq_temp=zeros(1,n_var_total); 
        Aeq_temp(insert_range1) =  t_vec(poly_order,1,1)'*Dn/dn;
        Aeq_temp(insert_range2) =  t_vec(poly_order,0,1)'*Dn_1/dn_1;
        
        % add equality constraint 
        Aeq=[Aeq ; Aeq_temp]; beq_x=[beq_x ; 0]; beq_y=[beq_y ; 0]; beq_z=[beq_z ; 0];
        insert_range1=insert_range2; insert_range2=insert_range1+poly_order+1;
    end
    
   % 2nd order continuity 
   insert_range1=1:poly_order+1; insert_range2=insert_range1+poly_order+1;
    
    for n=1:n_seg-1
        
        dn = ts(n+1)-ts(n);
        dn_1=ts(n+2)-ts(n+1);
        
        Dn=time_scailing_mat(ts(n),ts(n+1),poly_order); % Dn
        Dn_1=time_scailing_mat(ts(n),ts(n+1),poly_order);%Dn+1

        Aeq_temp=zeros(1,n_var_total); 
        Aeq_temp(insert_range1) =  t_vec(poly_order,1,1)'*Dn/dn^2;
        Aeq_temp(insert_range2) =  t_vec(poly_order,0,1)'*Dn_1/dn_1^2;
        
        % add equality constraint 
        Aeq=[Aeq ; Aeq_temp]; beq_x=[beq_x ; 0]; beq_y=[beq_y ; 0]; beq_z=[beq_z ; 0];
        insert_range1=insert_range2; insert_range2=insert_range1+poly_order+1;
    end
    
    %% QP solve   
    
    
end
