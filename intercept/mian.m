%% this is interception problem using numerical optimization 

%% Parameters
    global t_horizon_prediction t_horizon_observ n
    t_horizon_prediction=1;  % prediction horizon 
    t_horizon_observ=0.4;    % observation horizon 
    n=3; % poly order 
    N_obs=5;

%% Target intial observation 
    t_cur=0;
    ts=linspace(t_cur,t_cur+t_horizon_observ,N_obs);
    xs=0:4;
    ys=0:4;
    zs=0:4;
 
%% UAV intial state
    X=[1 0 0]';
    Xdot=[0 0 0];
    t_cur=t_horizon_observ;  % time elapesd 
    
%% Target traj estimation 
    [pt_x,pt_y,pt_z]=target_motion_estimate(ts,[xs;ys;zs]);  % coefficients from 0th order to nth order 
    
    
%% Optimization
    % Compute time vector 
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
    
    % Optimization parameters of inital 
    pr_x=zeros(n,1); pr_y=zeros(n,1); pr_z=zeros(n,1);
    pr_x(1)=X(1); pr_y(1)=X(2); pr_z(1)=X(3);
    
    % Optimization spec
    w0=1; w1=0.5; % weight for positional & velocity errors
    
    % Optimization (Output: polynomial scaled in [0 1])
    pr_x=quadprog(w0*T0+w1*T1,-w0*T0*pt_x-w1*T1*pt_x,[],[],[t0'; t1'],[X(1); Xdot(1)*(t_horizon_prediction)],[],[],pr_x);
    pr_y=quadprog(w0*T0+w1*T1,-w0*T0*pt_y-w1*T1*pt_y,[],[],[t0'; t1'],[X(2); Xdot(2)*(t_horizon_prediction)],[],[],pr_y);
    pr_z=quadprog(w0*T0+w1*T1,-w0*T0*pt_z-w1*T1*pt_z,[],[],[t0'; t1'],[X(3); Xdot(3)*(t_horizon_prediction)],[],[],pr_z);
    
    
 %% Analysis result 
    %% Target trajectory 
        % Observed trajectory 
        figure
        axis([0 10 0 10 0 10])
        hold on 
        plot3(xs,ys,zs,'b-','LineWidth',3)
        % Predicted trajectory 
        
 
    
    
    
    


    