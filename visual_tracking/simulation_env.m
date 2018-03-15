%% main file 
% map representation resolution 
map_res=0.5;
addpath('C:\Users\junbs\Documents\path_planinng/robot-10.1/')
% real map 
map.origin=zeros(3,1);
map.size=[20 20 20]; 
map.mat=zeros(floor(map.size(1)/map_res)+1,floor(map.size(2)/map_res)+1,floor(map.size(3)/map_res)+1);
N_obs=2; % number of obstacles 
obstacles=[];


% map rendering 
obs1=[10 10 7 3 3 1]; %[position + lengths/2] 
figure
hold on
xlabel('x'); ylabel('y'); zlabel('z')
patch([map.origin(1) map.origin(1)+map.size(1) map.origin(1)+map.size(1) map.origin(1)],[map.origin(2) map.origin(2) map.origin(2)+map.size(2) map.origin(2)+map.size(2) ],[0.1 0.2 0.1])
Rplot(zeros(1,3),eye(3),1,2);
axis([map.origin(1) map.origin(1)+map.size(1) map.origin(2) map.origin(2)+map.size(2) map.origin(3) map.origin(3)+map.size(2)])
% draw_box(obs1(1:3)-obs1(4:6),obs1(1:3)+obs1(4:6),'w',0.01)
draw_box(obs1(1:3)-obs1(4:6),obs1(1:3)+obs1(4:6),'k',0.3)


% Observer position 
p_obs=[3 3 10];
R_obs=rotz(pi/6)*roty(pi/6);
Rplot(p_obs,R_obs,1,2);
axis equal

obstacles=[obstacles ; obs1];

% encode the map info into voxel 

for idx=size(obstacles,1)
    obs=obstacles(idx,:);
    x_idx_lower=round((obs(1)-obs(4))/map_res);
    x_idx_upper=round((obs(1)+obs(4))/map_res);
    y_idx_lower=round((obs(2)-obs(5))/map_res);
    y_idx_upper=round((obs(2)+obs(5))/map_res);
    z_idx_lower=round((obs(3)-obs(6))/map_res);
    z_idx_upper=round((obs(3)+obs(6))/map_res);
end

map.mat(x_idx_lower:x_idx_upper,y_idx_lower:y_idx_upper,z_idx_lower:z_idx_upper)=1;  
map.mat(:,:,1)=1; %floor

% check in voxel representation 
% figure
% title('voxel representation')
% hold on
% xlabel('x'); ylabel('y'); zlabel('z')
% patch([map.origin(1) map.origin(1)+map.size(1) map.origin(1)+map.size(1) map.origin(1)],[map.origin(2) map.origin(2) map.origin(2)+map.size(2) map.origin(2)+map.size(2) ],[0.1 0.2 0.1])
% Rplot(zeros(1,3),eye(3),1,2);
% axis([map.origin(1) map.origin(1)+map.size(1) map.origin(2) map.origin(2)+map.size(2) map.origin(3) map.origin(3)+map.size(2)])
% 
% [i ,j ,k] = ind2sub(size(map.mat),find(map.mat == 1));
% search_zip=[i j k];
% for draw_idx=1:length(search_zip)
%     idx_x=search_zip(draw_idx,1); idx_y=search_zip(draw_idx,2); idx_z=search_zip(draw_idx,3);
%     p1=[map.origin(1)+map_res*(idx_x)-map_res/2 map.origin(2)+map_res*(idx_y)-map_res/2 map.origin(3)+map_res*(idx_z)-map_res/2];
%     p2=[map.origin(1)+map_res*(idx_x)+map_res/2  map.origin(2)+map_res*(idx_y)+map_res/2 map.origin(3)+map_res*(idx_z)+map_res/2];
%     draw_box(p1,p2,'c',0.9)
% end

% voxel map 
VoxMap.origin=map.origin;
VoxMap.size=map.size;
VoxMap.res=1; % resolution 
VoxMap.mat=-1*ones(floor(map.size(1)/map_res)+1,floor(map.size(2)/map_res)+1,floor(map.size(3)/map_res)+1); % unknown=-1

%% raycasting setting & sample point 

% sensor spec
FOV=2*pi/3;
FOD=20; 
n_ang=10;
angs=linspace(-FOV/2,FOV/2,n_ang);
end_pnts_tmp=[];
end_pnts=[];
for ang=angs
    end_pnts_tmp=[end_pnts_tmp [FOD*cos(ang) 0 FOD*sin(ang)]'];
end
end_pnts=[end_pnts end_pnts_tmp];
for ang=linspace(0,2*pi,n_ang)
    end_pnts=[end_pnts rotx(ang)*end_pnts_tmp];
end




%% raycasting from arbitrary pose
figure
title('mapping voxel')
hold on
xlabel('x'); ylabel('y'); zlabel('z')

Rplot(zeros(1,3),eye(3),1,2);
axis([map.origin(1) map.origin(1)+map.size(1) map.origin(2) map.origin(2)+map.size(2) map.origin(3) map.origin(3)+map.size(2)])

[i ,j ,k] = ind2sub(size(map.mat),find(map.mat == 1));
search_zip=[i j k];
for draw_idx=1:length(search_zip)
    idx_x=search_zip(draw_idx,1); idx_y=search_zip(draw_idx,2); idx_z=search_zip(draw_idx,3);
    p1=[map.origin(1)+map_res*(idx_x)-map_res/2 map.origin(2)+map_res*(idx_y)-map_res/2 map.origin(3)+map_res*(idx_z)-map_res/2];
    p2=[map.origin(1)+map_res*(idx_x)+map_res/2  map.origin(2)+map_res*(idx_y)+map_res/2 map.origin(3)+map_res*(idx_z)+map_res/2];
    draw_box(p1,p2,'g',0.2)
end


% observer pose
T=eye(4);
T(1:3,4)=p_obs;
T(1:3,1:3)=R_obs;
% inspection point 
end_pnts_observer=T*[end_pnts; ones(1,length(end_pnts))];
end_pnts_observer=end_pnts_observer(1:3,:);
end_pnts_observer=end_pnts_observer+p_obs'; % real inspection points
% observer axis
Rplot(p_obs,R_obs,2,2)
plot3(end_pnts_observer(1,:),end_pnts_observer(2,:),end_pnts_observer(3,:),'b*')
axis equal
% raycasting from the observer

for inspection=1:length(end_pnts_observer)
    
p1=p_obs'; p2=end_pnts_observer(:,inspection);
plot3(p2(1),p2(2),p2(3),'r*')
n_sample=10; % need to be tuned 
samples_x=linspace(p1(1),p2(1),n_sample);
samples_y=linspace(p1(2),p2(2),n_sample);
samples_z=linspace(p1(3),p2(3),n_sample);

for sample_idx=1:n_sample
    idx_intersect_x=floor((samples_x(sample_idx)-map.origin(1))/map_res);
    idx_intersect_y=floor((samples_y(sample_idx)-map.origin(2))/map_res);
    idx_intersect_z=floor((samples_z(sample_idx)-map.origin(3))/map_res);
    
    idx_intersect_x=max(idx_intersect_x,1);
    idx_intersect_y=max(idx_intersect_y,1);
    idx_intersect_z=max(idx_intersect_z,1);
    
    
    idx_intersect_x=min(idx_intersect_x,size(map.mat,1));
    idx_intersect_y=min(idx_intersect_y,size(map.mat,1));
    idx_intersect_z=min(idx_intersect_z,size(map.mat,1));
    
    
    if map.mat(idx_intersect_x,idx_intersect_y,idx_intersect_z)==1
        break
    end
    
end

p_intersect=[samples_x(sample_idx) samples_y(sample_idx) samples_z(sample_idx)];
VoxMap.mat(idx_intersect_x,idx_intersect_y,idx_intersect_z)=1;


[i ,j ,k] = ind2sub(size(VoxMap.mat),find(VoxMap.mat == 1));
search_zip=[i j k];
for draw_idx=1:size(search_zip,1)
    idx_x=search_zip(draw_idx,1); idx_y=search_zip(draw_idx,2); idx_z=search_zip(draw_idx,3);
    corner1=[map.origin(1)+map_res*(idx_x)-map_res/2 map.origin(2)+map_res*(idx_y)-map_res/2 map.origin(3)+map_res*(idx_z)-map_res/2];
    corner2=[map.origin(1)+map_res*(idx_x)+map_res/2  map.origin(2)+map_res*(idx_y)+map_res/2 map.origin(3)+map_res*(idx_z)+map_res/2];
    draw_box(corner1,corner2,'c',0.9)
end

end






