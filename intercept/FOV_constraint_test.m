%% Description 
% this script is for FOV constraint test
addpath('FOV_opti')
%% Parameters
    global t_horizon_prediction  n
    t_horizon_prediction=1;  % prediction horizon 
    n=7; 
    
    % Compute time vector t s.t p(t)=p*t
    t0=zeros(n+1,1); t1=zeros(n+1,1);
    t0(1)=1; t1(2)=1;

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

%% Target trajectory 
    % entire trajectory 
    N_sim=20;
    theta=linspace(-pi/2,pi/2,N_sim);
    R=5;
    xs=R*cos(theta)/3; ys=R*sin(theta); zs=linspace(0,3,N_sim);
    
    g_history=[xs;ys;zs]; % positional history of target
    
    % partially observed trajectory 
    
    N_obs=6; % observation number 
    
    t_obs=linspace(0,1,N_obs);
    xt=xs(1:N_obs); yt=ys(1:N_obs); zt=zs(1:N_obs);
    [pt_x,pt_y,pt_z]=target_motion_estimate(t_obs,[xt;yt;zt]);
%% UAV state
    X=[-2 -5 0]'; Xdot=[0 0 0];
    xr=X(1); yr=X(2); zr=X(3);
    xr_dot=0; yr_dot=0; zr_dot=0;
    
%% Opimization 
    % Augmented optimization parameters = P=[px ; py ; pz] ( 3(n+1) x 1 )

    % Optimization weights
    w0=60; w1=5; w3=0.002; % weight for positional & velocity errors % jerk 

    %% Initial paremeters obtaining 
    % set the solution w/o FOV constraint 

    % Optimization (Output: polynomial scaled in [0 1])
    pr_x=quadprog(w0*T0+w1*T1+w3*T3,-w0*T0*pt_x-w1*T1*pt_x,[],[],[t0'; t1'],[xr; xr_dot*(t_horizon_prediction)],[],[],[],options);
    pr_y=quadprog(w0*T0+w1*T1+w3*T3,-w0*T0*pt_y-w1*T1*pt_y,[],[],[t0'; t1'],[yr; yr_dot*(t_horizon_prediction)],[],[],[],options);
    pr_z=quadprog(w0*T0+w1*T1+w3*T3,-w0*T0*pt_z-w1*T1*pt_z,[],[],[t0'; t1'],[zr; zr_dot*(t_horizon_prediction)],[],[],[],options);

    %% QCQP solve for FOV constraint 
    pr=[pr_x ; pr_y ; pr_z]; % initial parameter 
    
    % obj function 
    Q=w0*blkdiag(T0,T0,T0)+w1*blkdiag(T1,T1,T1)+w3*blkdiag(T3,T3,T3); f=-(w0*[T0*pt_x ;T0*pt_y ;T0*pt_z]+w1*[T1*pt_x ;T1*pt_y ;T1*pt_z]);
    c=0;
    %%
    % linear constraint
    %%%%%%%%%%%%%%%%%%%
    A=[]; b=[]; H=[]; k=[]; d=[];
    % initial position 
    Aeq=blkdiag(t0',t0',t0'); beq=[xr yr zr]';
    % initial velocity 
    Aeq=[Aeq ; blkdiag(t1',t1',t1')]; beq=[beq [xr_dot yr_dot zr_dot]'];
    
    %%
    % Quadratic constraint (FOV)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % FOV contraint apply time 
    
    t_FOV_constr=[0.6 0.7 0.8 0.9 1];
    tol=repmat([0.01],1,length(t_FOV_constr));
    i=1;
    H=[]; k=[]; d=[];
    for t=t_FOV_constr
        % upper limit
        H{2*i-1}=2*blkdiag(t_vector(t,2)*t_vector(t,0)',t_vector(t,2)*t_vector(t,0)',t_vector(t,2)*t_vector(t,0)');
        H{2*i-1}=(H{2*i-1}+H{2*i-1}')/2;
        k{2*i-1}=-([pt_x'*t_vector(t,0)*t_vector(t,2)' pt_y'*t_vector(t,0)*t_vector(t,2)'...
            pt_z'*t_vector(t,0)*t_vector(t,2)'-9.81*t_vector(t,0)'])';
        d{2*i-1}=-9.81*t_vector(t,0)'*pt_z-tol(i);
        
        % lower limit
        H{2*i}=-2*blkdiag(t_vector(t,2)*t_vector(t,0)',t_vector(t,2)*t_vector(t,0)',t_vector(t,2)*t_vector(t,0)');
        H{2*i}=(H{2*i}+H{2*i}')/2;
        k{2*i}=([pt_x'*t_vector(t,0)*t_vector(t,2)' pt_y'*t_vector(t,0)*t_vector(t,2)' ...
            pt_z'*t_vector(t,0)*t_vector(t,2)'-9.81*t_vector(t,0)'])';
        d{2*i}=9.81*t_vector(t,0)'*pt_z-tol(i);
        
        i=i+1;
    end
    
   
    %% solve
     pr=QCQP(Q,f,c,H,k,d,A,b,Aeq,beq,pr);
    
    
    %% Analysis
    
    pr_x=pr(1:n+1); pr_y=pr(n+2:2*n+2); pr_z=pr(2*n+3:3*n+3);
    pr_x=fliplr(pr_x'); pr_y=fliplr(pr_y'); pr_z=fliplr(pr_z');
    pt_x=fliplr(pt_x'); pt_y=fliplr(pt_y'); pt_z=fliplr(pt_z');
    
    eval_t=linspace(0,1,10);
    % evaluated vehicle trajectory 
    xr=polyval(pr_x,eval_t); yr=polyval(pr_y,eval_t); zr=polyval(pr_z,eval_t);
    % target trajectory 
    xt_pred=polyval(pt_x,eval_t); yt_pred=polyval(pt_y,eval_t); zt_pred=polyval(pt_z,eval_t);
    
    %% plot 
    figure
    plot3(xr,yr,zr,'r*-')
    hold on 
    plot3(xs,ys,zs,'k--')
    plot3(xs(1:N_obs),ys(1:N_obs),zs(1:N_obs),'o')
    plot3(xt_pred,yt_pred,zt_pred,'g*-')
    hold off

    
    
 
    
    
    

    
    