%% Scenario
   for section=1
    %% Setting target path 
    N_sim=100; % total simulation step 
    t0=0; tf=10;
    dt=(tf-t0)/N_sim;
    t_sim=linspace(t0,tf,N_sim); % global time clock 
    
    theta=linspace(-pi/2,pi/2,N_sim/2);
    R=5;
    xs=R*cos(theta)/3; ys=R*sin(theta); zs=2-(t_sim(1:N_sim/2)-t_sim(N_sim/2)).^2;
    
    target_path=[xs;ys;zs]; % positional history of target
    
    xs=-xs; ys=ys+2*R; zs=3-(t_sim(N_sim/2+1:end)-t_sim(N_sim/2)).^2;
    
    target_path=[target_path [xs;ys;zs]];
    
    %% UAV inital position 
    p0=target_path(:,1)'+[-1 -2 3];
    v0=[0 0 0];
    a0=[0 0 0];
    
    %% check the target trajectory 
    figure
    plot3(target_path(1,:),target_path(2,:),target_path(3,:),'k:','LineWidth',2)
    hold on 
    plot3(p0(1),p0(2),p0(3),'r*','MarkerSize',8,'LineWidth',3)
    title('Target path and UAV initial pose')
    xlabel('x'); ylabel('y'); zlabel('z')
    
    end
%% Observation data 
for section=2
     addpath('FOV_opti')

    N_obs=7;
    obs_pnts=target_path(:,1:N_obs);   
    obs_t=t_sim(1:N_obs);

    plot3(obs_pnts(1,:),obs_pnts(2,:),obs_pnts(3,:),'bo','MarkerSize',8,'LineWidth',3)
    n=3;
    plot3(p0(1),p0(2),p0(3),'r*','MarkerSize',8,'LineWidth',3)
    %% Normal polyfit 
%     px=polyfit(obs_t,obs_pnts(1,:),n);
%     py=polyfit(obs_t,obs_pnts(2,:),n);
%     pz=polyfit(obs_t,obs_pnts(3,:),n);
% 
%     t_pred=linspace(0,1,20);
%     fit_pnts=[polyval(px,t_pred); polyval(py,t_pred) ;polyval(pz,t_pred) ];
%     plot3(fit_pnts(1,:),fit_pnts(2,:),fit_pnts(3,:),'g-','LineWidth',3)

    %% Modified polyfit

    ws=linspace(2,2,length(obs_pnts)); % regression weight 
    lambda=0.005; % penalty weight for curvature 
    obs_period=N_obs*dt; 
    pred_period=1.5*N_obs*dt;

    [px_t,res_x]=target_traj_fitting(obs_t,obs_pnts(1,:),n,ws,lambda,obs_period,pred_period);
    [py_t,res_y]=target_traj_fitting(obs_t,obs_pnts(2,:),n,ws,lambda,obs_period,pred_period);
    [pz_t,res_z]=target_traj_fitting(obs_t,obs_pnts(3,:),n,ws,lambda,obs_period,pred_period);

    t_pred=linspace(0,1,20); % prediction begins from 0 to 1 
    fit_pnts=[polyval(flip(px_t),t_pred); polyval(flip(py_t),t_pred) ;polyval(flip(pz_t),t_pred)];
    plot3(fit_pnts(1,:),fit_pnts(2,:),fit_pnts(3,:),'-','Color',[0 0 0 0.5],'LineWidth',3)

%     t_obs=linspace(-obs_period/pred_period,0,5);
%     fit_pnts=[polyval(px,t_obs); polyval(py,t_obs) ;polyval(pz,t_obs) ];
%     plot3(fit_pnts(1,:),fit_pnts(2,:),fit_pnts(3,:),'k-','LineWidth',3)


    %% Calculate intercept point using optimization 

    % degree of uncertainty 
    sum_res=res_x+res_y+res_z;

    w1=1000; w2=1; % weight of dou and jerk respectively 

    % interception points 
    tc=fmincon(@(t) w2*double(jerk_fun(px_t(1),px_t(2),px_t(3),px_t(4),a0(1),v0(1),p0(1),py_t(1),py_t(2),py_t(3),py_t(4),...
        a0(2),v0(2),p0(2),pz_t(1),pz_t(2),pz_t(3),pz_t(4),a0(3),v0(3),p0(3),t))+w1*double(dou_fun(sum_res,t)),0.5,[],[],[],[],0,1);
    pc=[fliplr(px_t')*t_vector(tc,0,n) ; fliplr(py_t')*t_vector(tc,0,n) ; fliplr(pz_t')*t_vector(tc,0,n)];
    vc=[fliplr(px_t')*t_vector(tc,1,n) ; fliplr(py_t')*t_vector(tc,1,n) ; fliplr(pz_t')*t_vector(tc,1,n)];

    plot3(pc(1),pc(2),pc(3),'Color',[0.5 0.2 0.2],'Marker','*','LineWidth',2,'MarkerSize',10)


end
%% Test the validity of interception point
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

%% Guidance
for section=3 
    n_pg=4; % poly order of guidance 
    Xr=p0; Vr=v0;
    %% method 1 : converge position error and velocity error     
    [px_r,py_r,pz_r]=guidance_path(px_t,py_t,pz_t,Xr,Vr,n_pg);
    t_pred=linspace(0,1,20); % prediction begins from 0 to 1 
    fit_pnts=[polyval(flip(px_r),t_pred); polyval(flip(py_r),t_pred) ;polyval(flip(pz_r),t_pred)];
    plot3(fit_pnts(1,:),fit_pnts(2,:),fit_pnts(3,:),'-','Color',[0 0 0.8 ],'LineWidth',3)
        
    %% method 2 : using interception point 
end




