%% ASAP SIMULATION 
% this includes the 4 basic scenarios all together  

% MAP SETTINGS
ws_range=[0 25 ; 0 15];
T1=SE2; T2=SE2; T3=SE2; T4=SE2;  
T1.t=[13 10]'; T2.t=[23 11]; T3.t=[10 3.6]; T4.t=[12 4];
obs1_scale=[6 3];  obs2_scale=[0.5 2];  obs3_scale=[1 1]; obs4_scale=[1 0.5];
obs1=obstacle2(T1,obs1_scale); obs2=obstacle2(T2,obs2_scale); obs3=obstacle2(T3,obs3_scale); obs4=obstacle2(T4,obs4_scale); 

% PATH MANAGER PARAMETERS 
path_parameters.inflate_rate=0.15; % inflate obstacle for more safety. but could make 
path_parameters.box_extension=0.1; % Increse : could consider unnecessary obstacles....
path_parameters.N_guide=1; % How many guide points every segment to avoid obstacle / 1 is proper in my case. for symmetry, use odd numbers  

% ASAP PARAMETERS 
dim=2; % Problem demension (2D)
w_v0=4; % Weight for visibility in graph path solving. if you increase this, ASAP will care travel distance more 
N_stride=20; % Stride number in one ray 
N_azim=30; % # of Azimuth sampling
max_ray_length=3; sample_ray_length=[3 2];
asap=ASAP_problem(dim,ws_range,{obs1,obs2,obs3,obs4},w_v0,N_stride,N_azim,max_ray_length,sample_ray_length,path_parameters);
asap.mapplot
hold on

% TRACKER 
tracker_position=[2.5 3];
% Tracker hisotry 
tracker_vel=[0 0];
% History of position and velocity of tracker 
tracker_path.x=[]; % position of smooth trajectory 
tracker_path.v=[]; % velocity of smooth trajectory 
tracker_path.waypoints={}; % waypoints to be generated 
tracker_path.refer={}; % referance target path to generate tracker path 

% TARGET
T_step=10;
target_path_x=[ linspace(5,19,T_step)  20*ones(1,T_step-2) linspace(18.5,12,T_step-4)  ]';
target_path_y=[  6*ones(1,T_step) linspace(7,14,T_step-2)  14*ones(1,T_step-4)]';
target_path=[target_path_x target_path_y];
h_target_path=plot(target_path(:,1),target_path(:,2),'rs-','LineWidth',2);
plot(tracker_position(1),tracker_position(2),'k^','LineWidth',2);% asap.mapplot

N_step=length(target_path);



%% SIMULATION PARAMS 
follow_ratio=4/5;
N_gen=0;

%% COMPARISON 
comparison_path = [];

t_ckp = [1 11 18 length(target_path)];
start_tracker_pos = tracker_position;

    

    x0 = start_tracker_pos;
    xf = [17.5 4]; 
    % okay proceed with this final position. 
    % Too big extension causes unnecessary guindace points
    Nx_grid=25;
    Ny_grid=25;

    left_lower_corner = [min(x0(1),xf(1)) min(x0(2),xf(2))]-[1 1];
    right_upper_corner = [max(x0(1),xf(1)) max(x0(2),xf(2))]+[1 1];


     waypoints = asap.PM.guidance_waypoint(start_tracker_pos,xf,left_lower_corner,right_upper_corner,Nx_grid,Ny_grid,11);

     
     comparison_path = [comparison_path ; waypoints];
     
     
     x0 = waypoints(end,:);
     xf = [20.5 9.5];
     
     left_lower_corner = [min(x0(1),xf(1)) min(x0(2),xf(2))]-[1 1];
    right_upper_corner = [max(x0(1),xf(1)) max(x0(2),xf(2))]+[1 1];

     
     waypoints = asap.PM.guidance_waypoint(x0,xf,left_lower_corner,right_upper_corner,Nx_grid,Ny_grid,7);

     
     
    comparison_path = [comparison_path ; waypoints];
     
    path_comp=plot([comparison_path(:,1) ;comparison_path(end,1)] ,[comparison_path(:,2) ;comparison_path(end,2)+0.5],'b-','LineWidth',3.5);
      
    
 
    for i = 1:length(comparison_path)
        
        if (i == 4 || i==5 || i == 6)
            
            quiver(comparison_path(i,1),comparison_path(i,2),target_path_x(i) - comparison_path(i,1) , target_path_y(i) - comparison_path(i,2),'b--','AutoScale','on','MaxHeadSize',0.4,'LineWidth',0.5) ;
        else
           arrow_comp = quiver(comparison_path(i,1),comparison_path(i,2),target_path_x(i) - comparison_path(i,1) , target_path_y(i) - comparison_path(i,2),'AutoScale','on','Color','b','MaxHeadSize',0.4,'LineWidth',0.8) ;

        end
        
    end
    
     quiver(comparison_path(i,1),comparison_path(i,2)+0.5,target_path_x(i+1) - comparison_path(i,1) , target_path_y(i+1) - (comparison_path(i,2)+0.5),'b--','AutoScale','on','MaxHeadSize',0.4,'LineWidth',0.5) ;
    
        
     
%% SIMULATION 

hold on 
generate_cond=true;
for t_step=2:N_step% traget movement step 
        h_target=plot(target_path(t_step,1),target_path(t_step,2),'rs-','LineWidth',3);
                       
        if generate_cond
           %% Predict and generate path 
           N_gen=N_gen+1; 
           N_pred=5; % Prediction horizon : total generation of waypoint = N_pred+1 
            prev_target_pos=target_path(t_step-1,:);
            cur_target_pos=target_path(t_step,:);        

             % Target prediction trajectory 
            dx=cur_target_pos(1)-prev_target_pos(1);
            dy=cur_target_pos(2)-prev_target_pos(2);        
            target_pred=[cur_target_pos];
            for horz=1:N_pred
                target_pred=[target_pred ; cur_target_pos+[dx*horz dy*horz]];
            end

            
            
            
            % Graph construction for key frame 
            asap.graph_init(tracker_position); 
            for i=1:N_pred+1
                asap.graph_insertion(target_pred(i,:))
            end        
            asap.graph_wrapper();       

            % Key frame proposal 
            waypoint=asap.path_proposal();    
            tracker_path.waypoints{N_gen}=waypoint;
            tracker_path.refer{N_gen}=target_pred;
            
            % Smooth path generation 
            N_iter=80;    % Iteration for path refinement      
            [xs_gen,vs_gen,xs_seg,vs_seg]=asap.PM.traj_gen(tracker_position,tracker_vel,waypoint,N_iter);    
            plot(xs_gen(:,1),xs_gen(:,2),'g--')
            N_exe=1; % execution seg step of generated trajectory 
        end           
                        
        % Follow path for one target step  (one segment)        
        for tracker_step=1:length(xs_seg{N_exe})
            tracker_path.x=[tracker_path.x ; xs_seg{N_exe}(tracker_step,:)];
            tracker_path.v=[tracker_path.v ; vs_seg{N_exe}(tracker_step,:)];
        end
        % Draw this segment 
        h_path=plot( xs_seg{N_exe}(1:length(xs_seg{N_exe}),1),xs_seg{N_exe}(1:length(xs_seg{N_exe}),2),'g-','LineWidth',4);

        % Re draw the waypoints of previous segment to overcame the overlap        
%         if N_gen>1
%             h_tracker=plot(tracker_path.waypoints{N_gen-1}(1:N_pred,1),tracker_path.waypoints{N_gen-1}(1:N_pred,2),'g^','MarkerSize',4,'LineWidth',2);       
%          end
        
        for seg=N_gen
%             plot(tracker_path.waypoints{seg}(1:N_exe,1),tracker_path.waypoints{seg}(1:N_exe,2),'g^','MarkerSize',4,'LineWidth',2)
            h_pic=quiver(tracker_path.waypoints{seg}(N_exe,1),...
                 tracker_path.waypoints{seg}(N_exe,2),...
                 target_path(t_step,1) - tracker_path.waypoints{seg}(N_exe,1),...
                 target_path(t_step,2) - tracker_path.waypoints{seg}(N_exe,2),'AutoScale','off','Color','g','MaxHeadSize',1,'LineWidth',1.5);
        end
        
        N_exe=N_exe+1;        

        
        % After following, we are here
        tracker_position=tracker_path.x(end,:); 
        tracker_vel=tracker_path.v(end,:);        
        
        % Path generation condition 
        % If it is too old or target deviates too much from prediction
        if N_exe>N_pred || norm(target_pred(N_exe-1,:) - target_path(t_step,:))>1
           generate_cond=true;               
        else
            generate_cond=false;               
        end
        
end


%% 
legend([h_target h_path path_comp],'Target','Proposed','W/O Visibility')
axis([0 25 0 20])
xlabel('x')
ylabel('y')
title('Tracking simulation','fontweight','bold','fontsize',16)





