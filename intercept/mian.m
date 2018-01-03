%% This is interception problem using numerical optimization 

%% Parameters
    global t_horizon_prediction t_horizon_observ n
    dt=0.1;  % time interval of each step 
    t_horizon_prediction=1.5;  % prediction horizon 
    N_obs=5; % observation fitting data number 
    t_horizon_observ=dt*N_obs;    % observation horizon                        
    t_exe=0.7;           % executed following horizon of UAV in real world (should be adjusted)
    tf=10;                   % final time of simulation 
    t_init=0;
    N_sim=length(t_init:dt:tf);
    n=4; % poly order 
    
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
         
%% Target 
    % target predifined history (C-curve) which UAV doesn't know 
    theta=linspace(-pi/2,pi/2,N_sim);
    R=5;
    xs=R*cos(theta)/3; ys=R*sin(theta); zs=linspace(0,3,N_sim);
    
    g_history=[xs;ys;zs]; % positional history of target
    target_traj_noise=rand(3,length(g_history))/10;
    g_history=g_history+target_traj_noise;
%     plot3(g_history(1,:),g_history(2,:),g_history(3,:),'b-')
    
% for now, lets assume UAV can observe initial movement 

%% UAV intial state    
    X=[-4 -5 1]'; Xdot=[0 0 0];
    xr=X(1); yr=X(2); zr=X(3);
    xr_dot=0; yr_dot=0; zr_dot=0;
    yaw=atan2((ys(1)-yr),(xs(1)-xr));
    p_history=[]; % positional history of UAV 
    yaw_history=[];        % yaw history of UAV
%% Main loop     
    replan=false; % replan start?  
    count=0; % iteration count
    count_replan=0;
    isPredicted=false; % is there any predicted motion? in the very first time, No. 
    % drawing 
    figure
    set(gcf,'Position',[100,300,800,500])
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
            
            % Optimization weights
            w0=30; w1=5; w3=1; % weight for positional & velocity errors % jerk 
            options=optimoptions('quadprog','Display','off');
            % Optimization (Output: polynomial scaled in [0 1])
            pr_x=quadprog(w0*T0+w1*T1+w3*T3,-w0*T0*pt_x-w1*T1*pt_x,[],[],[t0'; t1'],[xr; xr_dot*(t_horizon_prediction)],[],[],[],options);
            pr_y=quadprog(w0*T0+w1*T1+w3*T3,-w0*T0*pt_y-w1*T1*pt_y,[],[],[t0'; t1'],[yr; yr_dot*(t_horizon_prediction)],[],[],[],options);
            pr_z=quadprog(w0*T0+w1*T1+w3*T3,-w0*T0*pt_z-w1*T1*pt_z,[],[],[t0'; t1'],[zr; zr_dot*(t_horizon_prediction)],[],[],[],options);
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
            t_norm=(t-t_replan_start+dt)/t_horizon_prediction;
            xr=polyval(pr_x,t_norm); yr=polyval(pr_y,t_norm); zr=polyval(pr_z,t_norm);
            xr_dot=(xr-p_history(1,end))/dt; yr_dot=(yr-p_history(2,end))/dt; zr_dot=(zr-p_history(3,end))/dt;
            % predicted target state 
            xt_pred=polyval(pt_x,t_norm); yt_pred=polyval(pt_y,t_norm); zt_pred=polyval(pt_z,t_norm);
            % Desired yaw vector
            yaw=atan2(yt_pred-yr,xt_pred-xr); 
             
        end
        % save current state
        p_history=[p_history [xr; yr; zr]];
        yaw_history=[yaw_history yaw];
       %% do we need to replan? 
        count_replan=count_replan+1;
        count=count+1;
        
        fprintf('current iter : %d replanning count : %d \n',count,count_replan);
        
        if isPredicted
            replan=(count_replan==int32(t_exe/dt));  
        else
            replan=(count_replan==int32(t_horizon_observ/dt));  
        end
        
        
        %% Result animation 
        clf
        t_history=t_init:dt:t;

         % 3D traj 
        subplot(6,2,[2 4 6])
        title('3D traj')
        xlabel('x'); ylabel('y'); zlabel('z')  
        axis([min(xs)-5 max(xs)+2 min(ys)-5 max(ys)+2  min(zs)-3 max(zs)+2])
        hold on 
        % actual stastes
        plot3(g_history(1,1:count),g_history(2,1:count),g_history(3,1:count),'b-','LineWidth',1)         
        plot3(p_history(1,1:count),p_history(2,1:count),p_history(3,1:count),'k-','LineWidth',1);
        plot3(xr,yr,zr,'ko')
        plot3(g_history(1,count),g_history(2,count),g_history(3,count),'b*')
        if isPredicted
            % predicted target traj
            plot3(xs_pred,ys_pred,zs_pred,'g-','LineWidth',1)
            % planned UAV traj
            plot3(xrs,yrs,zrs,'r-','LineWidth',0.5)
            
        end
        hold off

           % xyz 
        subplot(6,2,[1 3])
        plot(t_history,g_history(1,1:count),t_history,p_history(1,1:count));
        axis([t_init tf min(g_history(1,:))-1 max(g_history(1,:))+1])
        xlabel('time[sec]'); ylabel('x')  

        subplot(6,2,[5 7])
        plot(t_history,g_history(2,1:count),t_history,p_history(2,1:count));
        axis([t_init tf min(g_history(2,:))-1 max(g_history(2,:))+1])
        xlabel('time[sec]'); ylabel('y')  

        subplot(6,2,[9 11])
        plot(t_history,g_history(3,1:count),t_history,p_history(3,1:count));
        axis([t_init tf min(g_history(3,:))-1 max(g_history(3,:))+1])
        xlabel('time[sec]'); ylabel('z')  

        % Image plane 
        % Camera spec 
        f=0.2; FOV=pi/3*2;
        % Calculate image on image plane ( small angle of roll of camera is assumed )
        
        d=sqrt((g_history(1,1:count)-p_history(1,1:count)).^2+(g_history(2,1:count)-p_history(2,1:count)).^2);
        alpha=yaw_history-atan2(g_history(2,1:count)-p_history(2,1:count),g_history(1,1:count)-p_history(1,1:count));
        xis=tan(alpha);
        yis=(g_history(3,1:count)-p_history(3,1:count))./(d.*cos(alpha));
        subplot(6,2,[8 10 12])
        title('Image plane')
        hold on
        axis([-tan(FOV/2) tan(FOV/2) -tan(FOV/2) tan(FOV/2)])
        plot(xis,yis,'k-','LineWidth',1)
        hold off
        
        
        pause(1e-1)
       
    end
 
    
%% Result analysis 
        t_history=t_init:dt:tf;
         % 3D traj 
        subplot(6,2,[2 4 6])
        title('3D traj')
        xlabel('x'); ylabel('y'); zlabel('z')  
        hold on 
        plot3(g_history(1,:),g_history(2,:),g_history(3,:),'b-','LineWidth',1)         
        plot3(p_history(1,:),p_history(2,:),p_history(3,:),'k-','LineWidth',1);
        hold off

           % xyz 
        subplot(6,2,[1 3])
        plot(t_history,g_history(1,:),t_history,p_history(1,:));
        xlabel('time[sec]'); ylabel('x')  

        subplot(6,2,[5 7])
        plot(t_history,g_history(2,:),t_history,p_history(2,:));
        xlabel('scaled time [elapsed/pred]'); ylabel('y')  

        subplot(6,2,[9 11])
        plot(t_history,g_history(3,:),t_history,p_history(3,:));
        xlabel('scaled time [elapsed/pred]'); ylabel('z')  

        % Image plane 
        % Camera spec 
        f=0.2; FOV=pi/3*2;
        
        % Calculate image on image plane ( small angle of roll of camera is assumed )
        
        d=sqrt((g_history(1,:)-p_history(1,:)).^2+(g_history(2,:)-p_history(2,:)).^2);
        alpha=yaw_history-atan2(g_history(2,:)-p_history(2,:),g_history(1,:)-p_history(1,:));
        xis=tan(alpha);
        yis=(g_history(3,:)-p_history(3,:))./(d.*cos(alpha));
        subplot(6,2,[8 10 12])
        hold on
        axis([-tan(FOV/2) tan(FOV/2) -tan(FOV/2) tan(FOV/2)])
        plot(xis,yis,'k-','LineWidth',1)
        hold off
        
        
    
    