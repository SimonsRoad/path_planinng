function [pr_x,pr_y,pr_z]=guidance_path(pt_x_in,pt_y_in,pt_z_in,Xr,Vr,Ar,nr,g,FOV_tol)
    %% this function takes the poly coefficients of target traj and initial condition  
    
    n=nr; % poly order of robot 
    w0=150; w1=20; w3=0.1; 
    xr=Xr(1); yr=Xr(2); zr=Xr(3); xr_dot=Vr(1); yr_dot=Vr(2); zr_dot=Vr(3);
    xr_ddot=Ar(1); yr_ddot=Ar(2); zr_ddot=Ar(3);
    % for the case : poly order of target < that of UAV
    pt_x=zeros(n+1,1); pt_x(1:length(pt_x_in))=pt_x_in; 
    pt_y=zeros(n+1,1); pt_y(1:length(pt_x_in))=pt_y_in;
    pt_z=zeros(n+1,1); pt_z(1:length(pt_x_in))=pt_z_in;
        
    % Compute time vector t s.t p(t)=p*t
    t0=zeros(n+1,1); t1=zeros(n+1,1); t2=zeros(n+1,1);
    t0(1)=1; t1(2)=1; t2(3)=1;

    % Compute the time integral matrix from 0 to 1
    T0=zeros(n+1,n+1);  
    T1=zeros(n+1,n+1);  % intergral of time vector 1st derviative    
    T3=zeros(n+1,n+1);  % jerk 
    for i=1:n+1
        for j=1:n+1
            T0(i,j)=1/(i+j-1);
            if i>1 && j>1
                T1(i,j)=(i-1)*(j-1)/(i+j-3);
            end
            if i>3 && j>3
                T3(i,j)=factorial(i-1)/factorial(i-4)*factorial(j-1)/factorial(j-4)/(i+j-8+1);
            end
        end
    end

    options=optimoptions('quadprog','Display','off');

    %% initial guess of polynomial 
    % Optimization (Output: polynomial scaled in [0 1])
    pr_x=quadprog(w0*T0+w1*T1+w3*T3,-w0*T0*pt_x-w1*T1*pt_x,[],[],[t0'; t1' ;t2'],[xr; xr_dot;xr_ddot],[],[],[],options);
    pr_y=quadprog(w0*T0+w1*T1+w3*T3,-w0*T0*pt_y-w1*T1*pt_y,[],[],[t0'; t1'; t2'],[yr; yr_dot;yr_ddot],[],[],[],options);
    pr_z=quadprog(w0*T0+w1*T1+w3*T3,-w0*T0*pt_z-w1*T1*pt_z,[],[],[t0'; t1'; t2'],[zr; zr_dot;zr_ddot],[],[],[],options);
    %% QCQP solve 
    pr=[pr_x ; pr_y ; pr_z]; % initial parameter 
    % obj function 
    Q=w0*blkdiag(T0,T0,T0)+w1*blkdiag(T1,T1,T1)+w3*blkdiag(T3,T3,T3); f=-(w0*[T0*pt_x ;T0*pt_y ;T0*pt_z]+w1*[T1*pt_x ;T1*pt_y ;T1*pt_z]);
    c=0;
    %% constraint
    % linear constraint
    %%%%%%%%%%%%%%%%%%%
    A=[]; b=[]; H=[]; k=[]; d=[];
    % initial position 
    Aeq=blkdiag(t0',t0',t0'); beq=[xr yr zr]';
    % initial velocity 
    Aeq=[Aeq ; blkdiag(t1',t1',t1')]; beq=[beq [xr_dot yr_dot zr_dot]'];
    % initial accel 
%     Aeq=[Aeq ; blkdiag(t2',t2',t2')]; beq=[beq [xr_ddot yr_ddot zr_ddot]'];

      
    %% Quadratic constraint (FOV)
    % FOV contraint apply time 
    
%     t_FOV_constr=linspace(0.1,1,5);
%     
%     tol=repmat([FOV_tol],1,length(t_FOV_constr));
%     i=1;
%     for t=t_FOV_constr
%         % upper limit
%         H{2*i-1}=2*blkdiag(t_vector(t,2,n)*t_vector(t,0,n)',t_vector(t,2,n)*t_vector(t,0,n)',t_vector(t,2,n)*t_vector(t,0,n)');
%         H{2*i-1}=(H{2*i-1}+H{2*i-1}')/2;
%         k{2*i-1}=-([pt_x'*t_vector(t,0,n)*t_vector(t,2,n)' pt_y'*t_vector(t,0,n)*t_vector(t,2,n)'...
%             pt_z'*t_vector(t,0,n)*t_vector(t,2,n)'-g*t_vector(t,0,n)'])';
%         d{2*i-1}=-g*t_vector(t,0,n)'*pt_z-tol(i);
%         
%         % lower limit
%         H{2*i}=-2*blkdiag(t_vector(t,2,n)*t_vector(t,0,n)',t_vector(t,2,n)*t_vector(t,0,n)',t_vector(t,2,n)*t_vector(t,0,n)');
%         H{2*i}=(H{2*i}+H{2*i}')/2;
%         k{2*i}=([pt_x'*t_vector(t,0,n)*t_vector(t,2,n)' pt_y'*t_vector(t,0,n)*t_vector(t,2,n)' ...
%             pt_z'*t_vector(t,0,n)*t_vector(t,2,n)'-g*t_vector(t,0,n)'])';
%         d{2*i}=g*t_vector(t,0,n)'*pt_z-tol(i);
%         
%         i=i+1;
%     end
%     
    %% QCQP solve
    pr=QCQP(Q,f,c,H,k,d,A,b,Aeq,beq,pr);
    pr_x=pr(1:n+1); pr_y=pr(n+2:2*n+2); pr_z=pr(2*n+3:3*n+3);

    
    
    
    