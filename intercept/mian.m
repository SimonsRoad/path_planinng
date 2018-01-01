%% This is interception problem using numerical optimization 

%% Parameters
    global t_horizon_prediction t_horizon_observ n
    t_horizon_prediction=1;  % prediction horizon 
    t_horizon_observ=0.5;    % observation horizon 
    dt=0.1;                  % time interval of each step   
    t_elapsed=0.7;           % executed following horizon of UAV in real world (should be adjusted)
    tf=10;                   % final time of simulation 
    t_init=0;
    N_sim=length(t_init:dt:tf);
    n=5; % poly order 
    N_obs=5; % observation fitting data number 
    
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
          
%% UAV intial state    
    X=[2 0 2]'; Xdot=[0 0 0];
    Yaw=0;   
%% history 
    p_history=X; % positional history of UAV 
    yaw_history=Yaw;        % yaw history of UAV
    
    % target predifined history (C-curve) which UAV doesn't know 
    theta=linspace(-pi/2,pi/2,N_sim);
    R=5;
    xs=R*cos(theta)/3; ys=R*sin(theta); zs=linspace(0,3,N_sim);
    g_history=[xs;ys;zs]; % positional history of target

% for now, lets assume UAV can observe initial movement 
%% Main loop     
    replan=false; % replan start?  
    count=1; % iteration count
    count_replan=1;
    isPredicted=false; % is there any predicted motion? in the very first time, No. 
    for t=t_init:dt:tf 
    %% replanning loop         
        if replan % If we need to replan (in this case, at every t_elapsed)      
        %% obtaining the latest observation data 
            t_obs=t-(N_obs-1)*dt:dt:t; % last N_obs time 
            xt_obs=xs(count-N_obs+1:count); yt_obs=ys(count-N_obs+1:count); zt_obs=zs(count-N_obs+1:count);
            t_replan_start=t; % start time of replanning. which correspond to 0 
        %% Target traj estimation     
            [pt_x,pt_y,pt_z]=target_motion_estimate(t_obs,[xt_obs;yt_obs;zt_obs]);  % coefficients from 0th order to nth order 
            isPredicted=true;
        %% Optimization
            % Optimization parameters of inital 
            pr_x=zeros(n,1); pr_y=zeros(n,1); pr_z=zeros(n,1);
            pr_x(1)=X(1); pr_y(1)=X(2); pr_z(1)=X(3);               % set parameters so that p(0)=X0

            % Optimization weights
            w0=50; w1=2; w3=0.001; % weight for positional & velocity errors % jerk 

            % Optimization (Output: polynomial scaled in [0 1])
            pr_x=quadprog(w0*T0+w1*T1+w3*T3,-w0*T0*pt_x-w1*T1*pt_x,[],[],[t0'; t1'],[X(1); Xdot(1)*(t_horizon_prediction)],[],[],pr_x);
            pr_y=quadprog(w0*T0+w1*T1+w3*T3,-w0*T0*pt_y-w1*T1*pt_y,[],[],[t0'; t1'],[X(2); Xdot(2)*(t_horizon_prediction)],[],[],pr_y);
            pr_z=quadprog(1.2*w0*T0+w1*T1+w3*T3,-1.2*w0*T0*pt_z-w1*T1*pt_z,[],[],[t0'; t1'],[X(3); Xdot(3)*(t_horizon_prediction)],[],[],pr_z);

            % For polyval(Matlab), we flip this coeff.
            pr_x=fliplr(pr_x'); pr_y=fliplr(pr_y'); pr_z=fliplr(pr_z');
            pt_x=fliplr(pt_x'); pt_y=fliplr(pt_y'); pt_z=fliplr(pt_z');

            % Reset the count to 0 
            count_replan=0;
        end
       %% Execute planned path 
        if isPredicted  % if no planning exits, we have nothing to execute
            %% prediction and corresponding plan
            t_eval=linspace(0,1,10); % in real time, this is [t_cur,t_cur+t_horizon_prediction] 
            % Predicted target traj
            xs_pred=polyval(pt_x,t_eval); ys_pred=polyval(pt_y,t_eval); zs_pred=polyval(pt_z,t_eval);
            xrs=polyval(pr_x,t_eval); yrs=polyval(pr_y,t_eval); zrs=polyval(pr_z,t_eval);

            %% Getting state
            % UAV planned state
            t_norm=(t-t_replan_start)/t_horizon_prediction;
            xr=polyval(pr_x,t_norm); yr=polyval(pr_y,t_norm); zr=polyval(pr_z,t_norm);
            % predicted target state 
            xt_pred=polyval(pt_x,t_norm); yt_pred=polyval(pt_y,t_norm); zt_pred=polyval(pt_z,t_norm);
            % Desired yaw vector
            yaw_d=atan2(yt_pred-yr,xt_pred-xr); 

            % save current state 
            p_history=[p_history [xr; yr; zr]];
        end
       %% do we need to replan? 
        count_replan=count_replan+1;
        count=count+1;
        if isPredicted
            replan=(count_replan==int32(t_elapsed/dt));  
        else
            replan=(count_replan==int32(t_horizon_observ/dt));  
        end
        
    end
 
    
%% Result 
        figure
        set(gcf,'Position',[100,300,800,500])
        %% 3D traj 
        subplot(2,2,4)
        title('3D traj')
        axis([0 15 0 15 0 15])
        xlabel('x'); ylabel('y'); zlabel('z')  
        hold on 
        plot3(xs,ys,zs,'b-','LineWidth',2) 
        plot3(xs_pred,ys_pred,zs_pred,'g*-','LineWidth',2);
        plot3(xrs,yrs,zrs,'k*-','LineWidth',2);
        quiver3(xrs,yrs,zrs,xs_pred-xrs,ys_pred-yrs,zs_pred-zrs,'r','LineWidth',0.5)
        hold off

        %% xyz 
        subplot(2,2,1)
        plot(t_eval,xs_pred,t_eval,xrs);
        xlabel('scaled time [elapsed/pred]'); ylabel('x')  


        subplot(2,2,2)
        plot(t_eval,ys_pred,t_eval,yrs);
        xlabel('scaled time [elapsed/pred]'); ylabel('y')  


        subplot(2,2,3)
        plot(t_eval,zs_pred,t_eval,zrs);
        xlabel('scaled time [elapsed/pred]'); ylabel('z')  

        %% Image plane 
        % Camera spec 
        f=0.2; FOV=pi/3*2;
        % Calculate image on image plane ( small angle of roll of camera is assumed )
        d=sqrt((xs_pred-xrs).^2+(ys_pred-yrs).^2);
        alpha=yaw_d-atan2((ys_pred-yrs),(xs_pred-xrs));
        xis=tan(alpha);
        yis=(zs_pred-zrs)./(d.*cos(alpha));
        figure
        hold on
        axis([-tan(FOV/2) tan(FOV/2) -tan(FOV/2) tan(FOV/2)])
        plot(xis,yis,'k-','LineWidth',3)
        hold off
        
        
    
    


    