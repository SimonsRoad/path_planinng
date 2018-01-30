%% Scenario
clc; clear all
   for section=1
    %% Setting target path 
    N_sim=200; % total simulation step 
    t0=0; tf=10;
    dt=(tf-t0)/N_sim;
    t_sim=linspace(t0,tf,N_sim); % global time clock 
    
    theta=linspace(-pi/2,pi/2,N_sim/2);
    R=6;
    xs=R*cos(theta)/3; ys=R*sin(theta); zs=-2*(t_sim(1:N_sim)-t_sim(1)).*(t_sim(1:N_sim)-t_sim(end))*(2/(tf-t0))^2;
    
    target_path=[xs;ys]; % positional history of target
    
    xs=-xs; ys=ys+2*R;
    target_path=[target_path [xs;ys]];
    target_path=[target_path ; zs];
    target_path=target_path;
    
    
    %% UAV inital position 
    p0=target_path(:,1)'+[-3 2 1];
    v0=[0 0 0];
    a0=[0 0 0];
    R0=eye(3);
    w0=[0 0 0]';
    state=[p0'; v0'; reshape(R0,9,1); w0];
    data.params.mQ = 0.5 ;
    data.params.J = diag([0.557, 0.557, 1.05]*1e-2);
    data.params.g = 9.81 ;
    data.params.e1 = [1;0;0] ;
    data.params.e2 = [0;1;0] ;
    data.params.e3 = [0;0;1] ;
    focal_l=0.05;
    odeopts = odeset('RelTol', 1e-4, 'AbsTol', 1e-5) ;
    
    %% check the target trajectory 
    figure
    set(gcf,'Position',[500 300 1200 600])
    subplot(1,2,1)
    plot3(target_path(1,:),target_path(2,:),target_path(3,:),'k:','LineWidth',2)
    hold on 
    plot3(p0(1),p0(2),p0(3),'r*','MarkerSize',8,'LineWidth',3)
    title('Target path and UAV initial pose')
    xlabel('x'); ylabel('y'); zlabel('z')
    axis equal
    axis_vector=[-3.5 10 -6 10 -5 5];
    axis(axis_vector)
    grid on 
    %% Setting 
    addpath('FOV_opti')
    do_fit=true; % should we perform fitting ? 
    replan=true;
    Xr=p0; Vr=v0; Ar=a0; % position and velocity of UAV 
    N_obs=10;
    obs_period=N_obs*dt;  % observation period 
    pred_period=1; % prediction period
    pred_acc_error_tol=2;  % accumulated prediction error toloerance  
   end
%% Simulation start 
X_target_history=[];
X_UAV_history=[]; V_UAV_history=[]; A_UAV_history=[];
f_UAV_history=[]; M_UAV_history=[]; proj_history=[];
b3_history=[];
t_history=[]; t_replan_history=[];
for step=N_obs:N_sim    
 %% Prediction and Planning
for section=2
        
    t_cur=t_sim(step); % current time 
    t_history=[t_history t_cur];
    X_target=target_path(:,step); % current target position 
    
    obs_pnts=target_path(:,step-N_obs+1:step);   
    obs_t=t_sim(step-N_obs+1:step);

    n_target=3; % target motion fitting poly order 
    %% Normal polyfit 
%     px=polyfit(obs_t,obs_pnts(1,:),n);
%     py=polyfit(obs_t,obs_pnts(2,:),n);
%     pz=polyfit(obs_t,obs_pnts(3,:),n);
% 
%     t_pred=linspace(0,1,20);
%     fit_pnts=[polyval(px,t_pred); polyval(py,t_pred) ;polyval(pz,t_pred) ];
%     plot3(fit_pnts(1,:),fit_pnts(2,:),fit_pnts(3,:),'g-','LineWidth',3)

    %% Modified polyfit
    if do_fit 
        fitting_error=0;
        t_fitting_start=t_cur;
       
        ws=linspace(2,2,length(obs_pnts)); % regression weight 
        lambda=0.001; % penalty weight for curvature 

        [px_t,res_x]=target_traj_fitting(obs_t,obs_pnts(1,:),n_target,ws,lambda,obs_period,pred_period);
        [py_t,res_y]=target_traj_fitting(obs_t,obs_pnts(2,:),n_target,ws,lambda,obs_period,pred_period);
        [pz_t,res_z]=target_traj_fitting(obs_t,obs_pnts(3,:),n_target,ws,lambda,obs_period,pred_period);
        do_fit=false;         
    end

    % poly fit error  
    t_exe=(t_cur-t_fitting_start)/pred_period; % evaluation point        
    pred_X_target=[(px_t')*t_vector(t_exe,0,n_target) ; (py_t')*t_vector(t_exe,0,n_target) ; (pz_t')*t_vector(t_exe,0,n_target)];
    fitting_error=fitting_error+norm(X_target-pred_X_target); 
    
    %% Calculate intercept point using optimization 

    % degree of uncertainty 
    sum_res=res_x+res_y+res_z;

    w1=1000; w2=1; % weight of dou and jerk respectively 

    % interception points 
%     tc=fmincon(@(t) w2*double(jerk_fun(px_t(1),px_t(2),px_t(3),px_t(4),a0(1),v0(1),p0(1),py_t(1),py_t(2),py_t(3),py_t(4),...
%         a0(2),v0(2),p0(2),pz_t(1),pz_t(2),pz_t(3),pz_t(4),a0(3),v0(3),p0(3),t))+w1*double(dou_fun(sum_res,t)),0.5,[],[],[],[],0,1);
%     pc=[(px_t')*t_vector(tc,0,n_target) ; (py_t')*t_vector(tc,0,n_target) ; (pz_t')*t_vector(tc,0,n_target)];
%     vc=[(px_t')*t_vector(tc,1,n_target) ; (py_t')*t_vector(tc,1,n_target) ; (pz_t')*t_vector(tc,1,n_target)];

%     plot3(pc(1),pc(2),pc(3),'Color',[0.5 0.2 0.2],'Marker','*','LineWidth',2,'MarkerSize',10)
    %Test the validity of interception point
    % ts=linspace(0.2,1,10);
    % jerks=[]; dous=[];
    % for i=ts
    % jerks=[jerks double(jerk_fun(px(1),px(2),px(3),px(4),a0(1),v0(1),p0(1),py(1),py(2),py(3),py(4),...
    %     a0(2),v0(2),p0(2),pz(1),pz(2),pz(3),pz(4),a0(3),v0(3),p0(3),i))];
    % dous=[dous dou_fun(sum_res,i)];
    % end
    % figure
    % subplot(3,1,1)
    % plot(ts,w2*jerks,'Color',[0 0 0.7])
    % subplot(3,1,2)
    % plot(ts,w1*dous,'Color',[0.7 0 0] )
    % subplot(3,1,3)
    % plot(ts,w1*dous+w2*jerks,'Color',[0 0 0])

end

%% Guidance
for section=3 
    n_pg=7; % poly order of guidance 
    %% method 1 : converge position error and velocity error     
    if replan % if we have to replan the guidance path  
        if isempty(A_UAV_history)
            FOV_tol=0.3;
        else
            FOV_tol_tmp=col_norm((A_UAV_history+repmat([0 0 9.81]',1,length(A_UAV_history)))).*col_norm(X_target_history-X_UAV_history);       
            FOV_tol=(mean(FOV_tol_tmp(max(end-N_obs,1):end))/max(FOV_tol_tmp(max(end-N_obs,1):end))/2)^3;            
        end
        
        % guidance path obtained 
        disp('path generation...')
        [px_r,py_r,pz_r] = guidance_path(px_t,py_t,pz_t,Xr,Vr*pred_period,Ar*pred_period^2,n_pg,9.81*pred_period^2,FOV_tol);
        
        % controller 
        disp('control input generation...')
        data.params.p=[px_r';py_r';pz_r'];
        data.params.pt=[px_t';py_t';pz_t'];
        [t, x] = ode15s(@UAV_dynamics_geometric_control, [0 1], state, odeopts, data) ; 
        
        t_buffer_start=t_cur;        
        replan=false;           
    end
    
    % guidance command
    t_exe=(t_cur+dt-t_buffer_start)/pred_period; % evaluation point         
    % no controller 
%     Xd=[(px_r')*t_vector(t_exe,0,n_pg) ; (py_r')*t_vector(t_exe,0,n_pg) ; (pz_r')*t_vector(t_exe,0,n_pg)];
%     Vd=[(px_r')*t_vector(t_exe,1,n_pg) ; (py_r')*t_vector(t_exe,1,n_pg) ; (pz_r')*t_vector(t_exe,1,n_pg)]/pred_period;
%     Ad=[(px_r')*t_vector(t_exe,2,n_pg) ; (py_r')*t_vector(t_exe,2,n_pg) ; (pz_r')*t_vector(t_exe,2,n_pg)]/pred_period^2;
%     Xr=Xd; 
%     Vr=Vd;
%     Ar=Ad;
%    
    % controller 
    %use guidance command 
    [~,min_idx]=min(abs(t-t_exe));    state=x(min_idx,:)';
    [dx,xd,f,M,b1d]=UAV_dynamics_geometric_control(t_exe,state,data);
    Xr=state(1:3); Vr=state(4:6)/pred_period; Ar=dx(4:6)/pred_period^2;
    Rr=reshape(state(7:15),3,3);
    b3_history=[b3_history Rr(:,3)];
    f_UAV_history=[f_UAV_history f];
    M_UAV_history=[M_UAV_history M'];

   
    % history  
    X_UAV_history=[X_UAV_history Xr];
    V_UAV_history=[V_UAV_history Vr];
    A_UAV_history=[A_UAV_history Ar];

    X_target_history=[X_target_history X_target];

    proj_history=[proj_history proj_image(focal_l,Xr,Rr,X_target)];
    
    %% method 2 : using interception point 
end
%% check the necessity of replan and fitting 
fprintf('current prediction error: %f\n',fitting_error)

if pred_acc_error_tol<fitting_error 
    do_fit=true; replan=true; 
    t_replan_history=[t_replan_history t_cur];
    disp('replanning activated')
end

%% Plotting 
for section=4
    % target path 
    subplot(1,2,2)
    plot(proj_history(1,2:end),proj_history(2,2:end),'ro-','LineWidth',2,'MarkerSize',2)
    axis([-focal_l*tan(60) focal_l*tan(60) -focal_l*tan(60) focal_l*tan(60)])
    title('image plane (120deg FOV)')

    
    subplot(1,2,1)
    title('traj')

    axis(axis_vector)

    hold on 
    plot3(target_path(1,:),target_path(2,:),target_path(3,:),'k:','LineWidth',2)
    

    % observation points 
    plot3(obs_pnts(1,:),obs_pnts(2,:),obs_pnts(3,:),'bo','MarkerSize',2,'LineWidth',3)

    % target current position 
    plot3(X_target(1),X_target(2),X_target(3),'r*','MarkerSize',8,'LineWidth',3)
              
    % UAV cur position 
    plot3(Xr(1),Xr(2),Xr(3),'k*','MarkerSize',8,'LineWidth',3)
    
    % future target traj
    t_pred=linspace(0,1,20); % prediction begins from 0 to 1 
    fit_pnts=[polyval(flip(px_t),t_pred); polyval(flip(py_t),t_pred) ;polyval(flip(pz_t),t_pred)];
    plot3(fit_pnts(1,:),fit_pnts(2,:),fit_pnts(3,:),'-','Color',[0 0 0 0.5],'LineWidth',2)
    
    % planned UAV traj 
    fit_pnts=[polyval(flip(px_r),t_pred); polyval(flip(py_r),t_pred) ;polyval(flip(pz_r),t_pred)];
    plot3(fit_pnts(1,:),fit_pnts(2,:),fit_pnts(3,:),'-','Color',[0 0.8 0 ],'LineWidth',3)
    
    % rotational matrix 
    Rplot(Xr,Rr,2)
    scale=5;
%     quiver3(Xr(1),Xr(2),Xr(3),scale*b1d(1),scale*b1d(2),scale*b1d(3))

    grid on     
    children = get(gca, 'children');
    pause(1e-1)
    delete(children(1:end))
   
end

    plot3(X_UAV_history(1,:),X_UAV_history(2,:),X_UAV_history(3,:),'ko-','LineWidth',1)

    plot3(X_target_history(1,:),X_target_history(2,:),X_target_history(3,:),'ro-','LineWidth',1)

if norm(Xr-X_target)<0.4
    break 
end

end

%% Result 
figure 
subplot(4,1,1)
plot(t_history,X_target_history(1,:))
hold on 
plot(t_history,X_UAV_history(1,:))
legend({'target','UAV'})
title('x history[m]')


subplot(4,1,2)
plot(t_history,X_target_history(2,:))
hold on 
plot(t_history,X_UAV_history(2,:))
legend({'target','UAV'})
title('y history[m]')

subplot(4,1,3)
plot(t_history,X_target_history(3,:))
hold on 
plot(t_history,X_UAV_history(3,:))
legend({'target','UAV'})
title('z history[m]')

subplot(4,1,4)
prod_history=A_UAV_history(1,:).*(X_target_history(1,:) - X_UAV_history(1,:))+...
    A_UAV_history(2,:).*(X_target_history(2,:) - X_UAV_history(2,:))+...
    (A_UAV_history(3,:)+9.81).*(X_target_history(3,:) - X_UAV_history(3,:));
cos_history=(prod_history./col_norm(A_UAV_history+repmat([0 0 9.81]',1,length(prod_history)))./col_norm(X_target_history-X_UAV_history));
plot(t_history,(acos(abs(cos_history)))*180/pi)
% plot(t_history,abs(acos(abs(cos_history))-pi/2)*180/pi)
hold on 
plot(t_replan_history,zeros(1,length(t_replan_history)),'ro','LineWidth',2)
plot(t_history,90*ones(1,length(t_history)),'r--')
legend({'','replan','90deg'})
title('ang btw b3 , d')

% 
% figure
% title('image plane')
% 
% plot(proj_history(1,2:end),proj_history(2,2:end),'ko-','LineWidth',3)
% axis([-focal_l*tan(60) focal_l*tan(60) -focal_l*tan(60) focal_l*tan(60)])

%% numerical stability ?
b3_tmp=A_UAV_history+repmat([0 0 9.81]',1,length(A_UAV_history));
test=[];
for i=1:length(b3_tmp)
    test=[test norm(b3_tmp(:,i)/norm(b3_tmp(:,i))-b3_history(:,i))];
end
test=real(test);

%% error vector 
error=X_target_history-X_UAV_history;
yaw=atan2(error(2,:),error(1,:))*180/pi;



