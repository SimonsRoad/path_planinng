%% real map contruction 
global real_map 
real_map=robotics.OccupancyGrid(10,10,20);

% map1

% obs_x=[3 7 8]; dx=[0.3 0.5 2];
% obs_y=[5 9 6]; dy=[2 1 0.5];
% Nx=200; Ny=200;
% Nobs=length(obs_x);
% 

% map2

obs_x=[7 6]; dx=[0.5 0.5];
obs_y=[2 8]; dy=[2 2];
Nx=200; Ny=200;
Nobs=length(obs_x);

for I =1: Nobs
    [xs,ys]=meshgrid(linspace(obs_x(I)-dx(I),obs_x(I)+dx(I),Nx),linspace(obs_y(I)-dy(I),obs_y(I)+dy(I),Ny));
    obstacle=[reshape(xs,[],1) reshape(ys,[],1)];
    setOccupancy(real_map,obstacle,1)
end



figure
real_map.show()

%% occu_map 

global occu_map 
occu_map=robotics.OccupancyGrid(10,10,20);

%% set occupancy value to location that is pre-defined 

% pre_range=2;
% pre_pose=[7 6 0];
% pre_Nray=200;
% 
% 
% upper_right=occu_map.world2grid(pre_pose(1:2)+pre_range*[1 1]);
% lower_left=occu_map.world2grid(pre_pose(1:2)+pre_range*[-1 -1]);
% mat=real_map.occupancyMatrix;
% 
% for ix=min(lower_left(1),upper_right(1)):max(lower_left(1),upper_right(1))
%     for iy=min(lower_left(2),upper_right(2)):max(lower_left(2),upper_right(2))
%         if abs(mat(ix,iy)-0.5)<0.1 % free
%             occu_map.setOccupancy([ix iy],0.1,'grid');
%         else
%             occu_map.setOccupancy([ix iy],0.9,'grid');
%         end
%     end
% end

%% initial pose 
x0=[1 2];

pose = [x0 0];
maxrange = 3;
Nray=100;

angle_min=-pi/8;
angle_max=pi/8;


% ray insertion using real_map and occu_map

rayinsertion(pose,angle_min,angle_max,Nray,maxrange)

occu_map.show()
hold on


N_init=100; % initial time step 
dt=0.1; 
K=20; % noisy trajectories 
max_iter=10; 
verbose=true;
tol=0.2;
xN=[9 2]';

% plot(x0(1),x0(2),'c*')
% hold on 
% plot(xN(1),xN(2),'r*')


[X0s,Y0s]=initial_guess_traj(x0,xN,N_init);



%%

occu_map.show()
hold on 
plot(x0(1),x0(2),'c*')
plot(xN(1),xN(2),'r*')
hold on 
Xs=[]; Ys=[]; costs=[];
for i=1:size(X0s,1)
    fprintf('current guess: %d ----------------------\n',i);
    [X_perturbed,Y_perturbed,X_history,Ys_history,cost]=STOMP_fun(X0s(i,:),Y0s(i,:),K,dt,100,0.5,verbose);
    Xs=[Xs;X_perturbed]; Ys=[Ys;Y_perturbed]; costs=[costs cost];
    plot(X_perturbed,Y_perturbed,'r-')
    pause(1)
end

%% move 

[~,max_idx]=min(costs);
X=Xs(max_idx,:); Y=Ys(max_idx,:);
figure
%%

n_move=10;
n_current=1;
n_interp=4; % number of interp between 2 points
%%

while n_current<N_init
is_colision=false;

for i=1:n_move
    
    n_current=n_current+1;
    
    pose=[X(n_current) Y(n_current) atan2(Y(n_current)-Y(n_current-1),X(n_current)-X(n_current-1))];
    % occupancy map update 
    rayinsertion(pose,angle_min,angle_max,Nray,maxrange)
    % re - evaluation
    
    [cost,~,obs_where]=cost_occupancy(X(n_current:end),Y(n_current:end),occu_map,{});
    obs_where=obs_where+(N_init-length(X(n_current:end)));
    
    % plotting 
    occu_map.show()
    hold on
    plot(X,Y,'b-')
    plot(pose(1),pose(2),'c*')
    plot(xN(1),xN(2),'r*')
    pause(1e-3)
    

    if cost>1000
        disp('obstacle encountered')
        is_colision=true;
        break
    end
        

    
end


if is_colision 

    %replanning 

    % exploration first 
    % frontier 

    obs_center=([X(obs_where) Y(obs_where)]);
    obs_rad=maxrange+2;

    frontiers=get_frontier(occu_map,obs_center,obs_rad);

    % frontier clustering 
    epsilon=0.2;
    MinPts=10;
    IDX=DBSCAN(frontiers,epsilon,MinPts);

    frontier_cluster_colors={'co','yo','mo'};
    frontier_centroid_colors={'c*','y*','m*'};

    N_cluster=max(IDX);
    centroids=[];
    for n=1:N_cluster
        this_cluster=frontiers(IDX==n,:);
        if length(this_cluster) > 30 
            centroid=mean(this_cluster);
            centroids=[centroids ; centroid];
        end
    end

    % let's look at the nearest centroid from agent and best information gain  
    w1=0.01;    
    w2=2;
    w3=2;
    
    
    centroid_dist=sqrt(sum((pose(1:2)-centroids).^2,2));
    
    centroid_info=compute_info_gain(centroids,maxrange,0.5);
    
    centroid_goal_dist=sqrt(sum(([xN(1) xN(2)]-centroids).^2,2));
    
    centroid_cost=w1*centroid_info-w2*centroid_dist-w3*centroid_goal_dist;
    
    [~,max_idx]=max(centroid_cost);
    
    max_centroid=centroids(max_idx,:);
    look_angle=atan2(max_centroid(2)-pose(2),max_centroid(1)-pose(1));
    Kp=0.3;
    
    % look at the centroid 
    
    while abs(pose(3)-look_angle)>5*pi/180
        pose(3)=pose(3)+Kp*(look_angle-pose(3));
        rayinsertion(pose,angle_min,angle_max,Nray,maxrange)
        
        % plotting 
        occu_map.show()
        hold on
        plot(X,Y,'b-')
        plot(pose(1),pose(2),'c*')
        plot(xN(1),xN(2),'r*')
        % total frontier 
        plot(frontiers(:,1),frontiers(:,2),'co')
        % target frontier 
        target_idx=find(IDX==max_idx);
        plot(frontiers(target_idx,1),frontiers(target_idx,2),'go')
        
        
        pause(1e-1)
        
    end
    
    % and go for some time 
    
    
    explore_target=(pose(1:2)+max_centroid)/2;
    N_explore=5;
    dx=(explore_target(1)-pose(1))/N_explore;
    dy=(explore_target(2)-pose(2))/N_explore;
    
    for n_explore=1:5
    pose(1:2)=pose(1:2)+[dx dy];
    rayinsertion(pose,angle_min,angle_max,Nray,maxrange)

    
        % explore 
        occu_map.show()
        hold on
        plot(X,Y,'b-')
        plot(pose(1),pose(2),'c*')
        plot(xN(1),xN(2),'r*')
        % total frontier 
        plot(frontiers(:,1),frontiers(:,2),'co')
        % target frontier 
        target_idx=find(IDX==max_idx);
        plot(frontiers(target_idx,1),frontiers(target_idx,2),'go')
        plot(explore_target(1),explore_target(2),'ro')
        pause(1e-1)
    
    end
    
    
    % replanning
    
    % using path from previous step 
    %X0=X(n_current:end); Y0=Y(n_current:end);
    
%     n_start_replan=floor((n_current+obs_where)/2);
    
    [X0s,Y0s]=initial_guess_traj([pose(1) pose(2)],xN,N_init);
    
    Xs=[]; Ys=[]; costs=[];
    for i=1:size(X0s,1)
        fprintf('current guess: %d ----------------------\n',i);
        [X_perturbed,Y_perturbed,X_history,Ys_history,cost]=STOMP_fun(X0s(i,:),Y0s(i,:),K,dt,100,1,verbose);
        Xs=[Xs;X_perturbed]; Ys=[Ys;Y_perturbed]; costs=[costs cost];
        plot(X_perturbed,Y_perturbed,'r-')
    end
        
    [~,max_idx]=min(costs);
    X_re=Xs(max_idx,:); Y_re=Ys(max_idx,:);
    
    X=X_re;
    Y=Y_re;
    
    n_current=1;
    
    % plot path...
    plot(X,Y,'b-')
    plot(X(1),Y(1),'c*')
    plot(xN(1),xN(2),'r*')

    
end
end
