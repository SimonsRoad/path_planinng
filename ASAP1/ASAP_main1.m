%% Probelm settings
% target position 
target=[5 6]'; tracker_position=[2.5 4];

% basic parameters 
dim=2;  w_v0=4; N_stride=20; N_azim=25; max_ray_length=3; sample_ray_length=[4 3 2];

% map scale 
ws_range=[0 10 ; 0 10];

% set obstacle shape and position

%% scenario 1
% target_path=[5 6.5 ; 6 6.5 ; 7 6.5 ; 8 6.5];
% T1=SE2; 
% T1.t=[5 8]';
% obs1_scale=[3 0.5];  
% obs1=obstacle2(T1,obs1_scale); 
% asap=ASAP_problem(dim,ws_range,{obs1},w_v0,N_stride,N_azim,max_ray_length,sample_ray_length);

%% scenario 2
% target_path=[3 6.5 ; 4 6.5 ; 5 6.5 ; 6 6.5];
% T1=SE2;   T2=SE2;
% T1.t=[5 8]'; T2.t=[5 6]';
% obs1_scale=[3 0.5];  obs2_scale=[0.3 0.2];
% obs1=obstacle2(T1,obs1_scale); obs2=obstacle2(T2,obs2_scale);
% asap=ASAP_problem(dim,ws_range,{obs1,obs2},w_v0,N_stride,N_azim,max_ray_length,sample_ray_length);
%  
%% scenario 3
% 
% target_path=[3 6.5 ; 4 6.5 ; 5 6.5 ; 6 6.5];
% tracker_position=[2.5 3];
% T1=SE2;   T2=SE2;
% T1.t=[5 8]'; T2.t=[5 4]';
% obs1_scale=[3 0.5];  obs2_scale=[0.5 0.2];
% obs1=obstacle2(T1,obs1_scale); obs2=obstacle2(T2,obs2_scale);
% asap=ASAP_problem(dim,ws_range,{obs1,obs2},w_v0,N_stride,N_azim,max_ray_length,sample_ray_length);
 

%% scenario 4
% target_path=[5 6.5 ; 6 6.5 ; 7 6.5 ; 7 6.5];
% T1=SE2; T2=SE2; T3=SE2; 
% T1.t=[5 8]'; T2.t=[6,5.3]; T3.t=[8 6.5];
% obs1_scale=[3 0.5];  obs2_scale=[2 0.3];  obs3_scale=[0.5 2];
% obs1=obstacle2(T1,obs1_scale); obs2=obstacle2(T2,obs2_scale); obs3=obstacle2(T3,obs3_scale); 
% asap=ASAP_problem(dim,ws_range,{obs1,obs2,obs3},w_v0,N_stride,N_azim,max_ray_length,sample_ray_length);

%% simulation env 
% this includes the 4 scenarios all together  

% map scale 
ws_range=[0 25 ; 0 15];
tracker_position=[2.5 4];

T1=SE2; T2=SE2; T3=SE2; T4=SE2;  
T1.t=[11 10]'; T2.t=[23 11]; T3.t=[7 5]; T4.t=[14 3];
obs1_scale=[8 3];  obs2_scale=[0.5 2];  obs3_scale=[1 0.5]; obs4_scale=[1 0.5];
obs1=obstacle2(T1,obs1_scale); obs2=obstacle2(T2,obs2_scale); obs3=obstacle2(T3,obs3_scale); obs4=obstacle2(T4,obs4_scale); 

% basic parameters 
dim=2;  w_v0=4; N_stride=20; N_azim=25; max_ray_length=3; sample_ray_length=[4 3 2];

asap=ASAP_problem(dim,ws_range,{obs1,obs2,obs3,obs4},w_v0,N_stride,N_azim,max_ray_length,sample_ray_length);
asap.mapplot
hold on

% target path
T_step=10;
target_path_x=[ linspace(5,19,T_step)  20*ones(1,T_step-2) ]';
target_path_y=[  6*ones(1,T_step) linspace(7,13,T_step-2)  ]';
target_path=[target_path_x target_path_y];
plot(target_path(:,1),target_path(:,2),'rs-');
plot(tracker_position(1),tracker_position(2),'k^','LineWidth',2);% asap.mapplot


%% Graph construction 
% [layer_names,layer_poses,layer_vises]=asap.layer_calc(target,1);
asap.graph_init(tracker_position);
asap.graph_insertion(target_path(1,:));
asap.graph_insertion(target_path(2,:));
asap.graph_insertion(target_path(3,:));
asap.graph_insertion(target_path(4,:));
asap.graph_wrapper();
path=asap.path_proposal();

%% local planning 
[xs,vs]=asap.PM.traj_gen(tracker_position,[0 0],path);

%% plot 

figure
asap.mapplot % obstacle 
hold on
plot(xs(:,1),xs(:,2),'g-','LineWidth',4)

plot(target_path(:,1),target_path(:,2),'r*-');
plot(tracker_position(1),tracker_position(2),'ko','LineWidth',2);% asap.mapplot

% candidates 
shapes={'s','d','*','x'};

for i=1:length(asap.layer_info_pose)
    for j=1:length(asap.layer_info_pose{i})
        candid_pose=asap.layer_info_pose{i}{j};        
        plot(candid_pose(1),candid_pose(2),strcat('k',shapes{i}),'MarkerSize',6)
    end
end

plot([tracker_position(1); path(:,1)],[tracker_position(2) ;path(:,2)],'ms','MarkerSize',12,'LineWidth',2);





