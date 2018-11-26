%% MINIMUM JERK TARJECTORY GENERATION (SOFT VERSION : WAYPOINTS INEQ )

function [pxs,pys,pzs]=min_jerk_ineq(ts,X0,Xdot0,Xddot0,waypoint_polygon_seq,corridor_polygon_seq)

% INPUT 
% = [ts,xs] (waypoint and corresponding time) 
% h th waypoint_polygon: polygon constraint for h th waypoint 
% h th corridor_polygon: corridor constraint for  h-1 ~ h waypoint. 
% N_div : how many inter-points will be considered between h-1 ~ h waypoint
% e.g is N_div = 1, (t_(h-1) + t_(h))/2 is constrained to be insider h th
% corridor polygon 
    % optimization param = [pX1' pY1' pZ1' ; pX2 pY2 pZ2 ; ...]

% OUTPUT 
% pxs = (poly_order + 1) x n_seg
    
    x0 = X0(1);
    y0 = X0(2);
    z0 = X0(3);
       
    xdot0 = Xdot0(1);
    ydot0 = Xdot0(2);
    zdot0 = Xdot0(3);
    
    xddot0 = Xddot0(1);
    yddot0 = Xddot0(2);
    zddot0 = Xddot0(3);
        
    n_seg=length(ts)-1; % future step
    poly_order=5; % for jerk, order 5 is enough 
   
    n_var = (poly_order+1) * n_seg * 3; % length of optimization variables
    
    % QP FORM : p'Qp+Hp
    
    %% construct collective jerk matrix 
    
    n_var_total=(poly_order+1)*n_seg; % total number of optimization variables 
    Q_jerk=zeros((poly_order+1)*n_seg);
   
    for n=1:n_seg
        % time scaling constant        
        Dn=time_scailing_mat(ts(n),ts(n+1),poly_order);
        dn=ts(n+1)-ts(n);
        Q_jerk((poly_order+1)*(n-1)+1:(poly_order+1)*n,(poly_order+1)*(n-1)+1:(poly_order+1)*n)=Dn*integral_jerk_squared(poly_order)*Dn/dn^5;
    end
    
    
    
    %% inequality constraint - waypoint polygon 
    
    % Aineq = Aineq1 x Aineq2
    % compute Aineq1 
    Aineq1 = [];
    bineq = []; 
        
    for h = 1:n_seg
        Aineq1 = blkdiag(Aineq1,waypoint_polygon_seq{h}.A);
        bineq = [bineq ; waypoint_polygon_seq{h}.b];
    end
    
    % compute Aineq2
    Aineq2 = [];
    for h = 1:n_seg
        Aineq2_blk = t_vec(poly_order,1,0)'*time_scailing_mat(ts(h),ts(h+1),poly_order);        
        for i=1:3
        Aineq2 = blkdiag(Aineq2,Aineq2_blk);
        end
    end

    Aineq = Aineq1 * Aineq2;
    
    
    %% inequality constraint - corridor polygon     
    % Aineq = Aineq1 x Aineq2
    % compute Aineq1 
    Aineq1 = [];
    for h = 1:n_seg
        Aineq1 = blkdiag(Aineq1,corridor_polygon_seq{h}.A);
        bineq = [bineq ; corridor_polygon_seq{h}.b];
    end
    
    % compute Aineq2
    Aineq2 = [];
    for h = 1:n_seg
         Aineq2_blk = t_vec(poly_order,0.5,0)'*time_scailing_mat(ts(h),ts(h+1),poly_order);        
        for i=1:3
         Aineq2 = blkdiag(Aineq2,Aineq2_blk);
        end
    end
    
    Aineq = [Aineq; Aineq1 * Aineq2];
   
    
    %% equality constratint 
    
    Aeq=zeros(3,n_var_total);  beq_x  = zeros(3,1); beq_y = zeros(3,1); beq_z  = zeros(3,1);
    % initial constraint 
    % x0
    Aeq(1,1)=1; beq_x(1)= x0(1); beq_y(1)=y0(1); beq_z(1)=z0(1);
    % v0
    Aeq(2,2)=1; beq_x(2)=xdot0; beq_y(2)=ydot0; beq_z(2)=zdot0;
    % a0 
    Aeq(3,3)=2; beq_x(2)=xddot0; beq_y(2)=yddot0; beq_z(2)=zddot0;

           
    % 0st order continuity 
    insert_range1=1:poly_order+1; insert_range2=insert_range1+poly_order+1;
    
    for n=1:n_seg-1
        Dn=time_scailing_mat(ts(n),ts(n+1),poly_order); % Dn
        Dn_1=time_scailing_mat(ts(n+1),ts(n+2),poly_order);%Dn+1

        Aeq_temp=zeros(1,n_var_total); 
        Aeq_temp(insert_range1) =  t_vec(poly_order,1,0)'*Dn;
        Aeq_temp(insert_range2) =  -t_vec(poly_order,0,0)'*Dn_1; 
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
        Dn_1=time_scailing_mat(ts(n+1),ts(n+2),poly_order); %Dn+1

        Aeq_temp=zeros(1,n_var_total); 
        Aeq_temp(insert_range1) =  t_vec(poly_order,1,1)'*Dn/dn;
        Aeq_temp(insert_range2) =  -t_vec(poly_order,0,1)'*Dn_1/dn_1;
        
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
        Dn_1=time_scailing_mat(ts(n+1),ts(n+2),poly_order);%Dn+1

        Aeq_temp=zeros(1,n_var_total); 
        Aeq_temp(insert_range1) =  t_vec(poly_order,1,2)'*Dn/dn^2;
        Aeq_temp(insert_range2) =  -t_vec(poly_order,0,2)'*Dn_1/dn_1^2;
        
        % add equality constraint 
        Aeq=[Aeq ; Aeq_temp]; beq_x=[beq_x ; 0]; beq_y=[beq_y ; 0]; beq_z=[beq_z ; 0];
        insert_range1=insert_range2; insert_range2=insert_range1+poly_order+1;
    end
   
    % equality constraint re arrange 
    Aeq_temp = Aeq;
    row_stride = size(Aeq,1);
    col_stride = 3*(poly_order + 1);
    Aeq = zeros(3*row_stride,n_var);
    
    for n = 1:n_seg
        Aeq(1:row_stride , col_stride*(n-1)+1:col_stride*(n-1)+1 + (poly_order+1) -1) =...
            Aeq_temp(:, (n-1)*(poly_order+1)+1:n*(poly_order+1));
        
        Aeq(row_stride +1 :2*row_stride , col_stride*(n-1)+1 + poly_order+1 : col_stride*(n-1)+1 + 2*(poly_order+1) -1) =...
            Aeq_temp(:, (n-1)*(poly_order+1)+1:n*(poly_order+1));
        
        Aeq(2*row_stride +1 : 3*row_stride, col_stride*(n-1)+1 + 2*(poly_order+1) : col_stride*(n-1)+1 + 3*(poly_order+1) -1 ) =...
            Aeq_temp(:, (n-1)*(poly_order+1)+1:n*(poly_order+1));
    end
    
    beq = [beq_x;beq_y;beq_z];
                  
    
    %% QP formulation 
    % objective function
    Q = blkdiag(Q_jerk,Q_jerk,Q_jerk); % jerk matrix
    
%     sol =quadprog(2*Q,[],Aineq,bineq,Aeq,beq);
     sol =quadprog(2*Q,[],Aineq,bineq,Aeq,beq);
      
     
     sol = reshape(sol,3*(poly_order+1),[]);
     
     pxs = sol(1:poly_order+1,:);     
     pys = sol(poly_order+1 +1 : 2*(poly_order+1),:);
     pzs = sol(2*(poly_order+1) + 1 : 3*(poly_order+1),:);
    
%     sol =quadprog(2*Q,[],[],[],Aeq,beq);

                  
    
end
