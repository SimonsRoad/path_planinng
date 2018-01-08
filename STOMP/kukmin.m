lx=10; ly=10; 
global real_map 

obs_x=[7 5 2 8]; scale_x=[3 0.2 2 2];
obs_y=[6.6 9 4 1.5]; scale_y=[0.2 1 0.3 1.5];

global dx dy 
global Nx Ny
Nx=50; Ny=50;
real_map=robotics.OccupancyGrid(lx,ly,Nx/lx);

dx=lx/Nx; dy=ly/Ny;

Nobs=length(obs_x);

for I =1:Nobs
    [xs,ys]=meshgrid(linspace(obs_x(I)-scale_x(I),obs_x(I)+scale_x(I),Nx),linspace(obs_y(I)-scale_y(I),obs_y(I)+scale_y(I),Ny));
    obstacle=[reshape(xs,[],1) reshape(ys,[],1)];
    setOccupancy(real_map,obstacle,1)
end


figure
real_map.show()

%% occu_map 

global occu_map 
occu_map=robotics.OccupancyGrid(lx,ly,Nx/lx);

% initialization with uncetain information 
maxrange = 3;
Nray=500;

angle_min=0;
angle_max=2*pi;


% agent1
x10=[2 0.5];

pose1 = [x10 pi/2];


% ray insertion using real_map and occu_map

rayinsertion(pose1,angle_min,angle_max,Nray,maxrange)

% agent2
x20=[4 1];
% x20=[8 8];

pose2 = [x20 pi/2];


% ray insertion using real_map and occu_map

rayinsertion(pose2,angle_min,angle_max,Nray,maxrange)

obstacle_accumulation=[];
[obstacle_i,obstacle_j]=find(occu_map.occupancyMatrix>0.7);

obstacle_accumulation=[obstacle_accumulation; [obstacle_i obstacle_j] ];

if ~isempty(obstacle_accumulation)
    occu_map.setOccupancy(obstacle_accumulation,1,'grid')
end



occu_map.show()
hold on
plot(x10(1),x10(2),'ro')
plot(x20(1),x20(2),'bo')

x1=x10; x2=x20;
%% Frontier
frontiers=get_frontier(occu_map,[5 5],20);

% frontier clustering 
epsilon=0.3;
MinPts=4;
n_drop=10;

c=['c','g','y','r','m'];

IDX=DBSCAN(frontiers,epsilon,MinPts);

N_cluster=max(IDX);
% filtered index
fil_IDX=[];
centroids=[];
n_valid_cluster=0;

for n=1:N_cluster
    this_cluster=frontiers(IDX==n,:);
    
    if length(this_cluster) > n_drop
        n_valid_cluster=n_valid_cluster+1;
        plot(frontiers(:,1),frontiers(:,2),'s','MarkerSize',0.5,'LineWidth',5)

        fil_IDX=[fil_IDX n];
        centroid=mean(this_cluster);
        centroids=[centroids ; centroid];
    end
end

[c1,c2]=assign_cent(x1,x2,centroids,frontiers,IDX,fil_IDX);

% plot(frontiers(:,1),frontiers(:,2),'co','MarkerSize',1,'LineWidth',2)
plot(centroids(:,1),centroids(:,2),'go','LineWidth',2)
plot(c1(1),c1(2),'r*')
plot(c2(1),c2(2),'b*')
%% Local planner

local_path1=local_planner(occu_map,x1,c1);
plot(local_path1(:,1),local_path1(:,2),'r--')

local_path2=local_planner(occu_map,x2,c2);
plot(local_path2(:,1),local_path2(:,2),'b--')

%% move 

n1_count=1; n1_exe_max=6; 
n2_count=1; n2_exe_max=6;

for t=1:100
    % move
    x1=local_path1(end-n1_count+1,:);
    x2=local_path2(end-n2_count+1,:);
    
    rayinsertion([x1 0],angle_min,angle_max,Nray,maxrange)
    rayinsertion([x2 0],angle_min,angle_max,Nray,maxrange)


    
    
    [obstacle_i,obstacle_j]=find(occu_map.occupancyMatrix>0.7);

    obstacle_accumulation=[obstacle_accumulation; [obstacle_i obstacle_j] ];

    if ~isempty(obstacle_accumulation)
        occu_map.setOccupancy(obstacle_accumulation,1,'grid')
    end
    
    occu_map.show()


   
    frontiers=get_frontier(occu_map,[5 5],20);
    
    IDX=DBSCAN(frontiers,epsilon,MinPts);

    N_cluster=max(IDX);
    % filtered index
    fil_IDX=[];
    centroids=[];
    
    n_valid_cluster=0;
    for n=1:N_cluster
        this_cluster=frontiers(IDX==n,:);
        

        if length(this_cluster) > n_drop
            n_valid_cluster=n_valid_cluster+1;
            plot(this_cluster(:,1),this_cluster(:,2),strcat(c(n_valid_cluster),'s'),'MarkerSize',0.5,'LineWidth',5)
            fil_IDX=[fil_IDX n];
            centroid=mean(this_cluster);
            centroids=[centroids ; centroid];
        end
    end

    if ~isempty(fil_IDX)
        [c1,c2]=assign_cent(x1,x2,centroids,frontiers,IDX,fil_IDX);
         plot(centroids(:,1),centroids(:,2),'go','LineWidth',2)

    end
           
    plot(c1(1),c1(2),'r*')
    plot(c2(1),c2(2),'b*')
    
    if n1_count==min(n1_exe_max,length(local_path1)-1)
       fprintf('agent1 reached way point. new local planner will be given\n')
       local_path1=local_planner(occu_map,x1,c1);
       n1_count=1;
    else
        n1_count=n1_count+1;
        
    end
    
    
    if n2_count==min(n2_exe_max,length(local_path2)-1)
       fprintf('agent2 reached way point. new local planner will be given\n')
       local_path2=local_planner(occu_map,x2,c2);
       n2_count=1;
       
    else
        n2_count=n2_count+1;
        
    end
    
    fprintf('agent1 : %d agent2 : %d \n',n1_count,n2_count);

   
    plot(x1(1),x1(2),'ro')
    plot(x2(1),x2(2),'bo')

   
%     plot(frontiers(:,1),frontiers(:,2),'co','MarkerSize',1,'LineWidth',2)

    
    plot(local_path1(:,1),local_path1(:,2),'r--')
    plot(local_path2(:,1),local_path2(:,2),'b--')

   
    pause(1e-1)

    
 
end

