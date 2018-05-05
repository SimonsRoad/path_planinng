%% Probelm settings
% 2D 
dim=2; 
% map scale 
range=[0 10 ; 0 10];
% set obstacle shape and position
T1=SE2; T2=SE2;
T1.t=[5 8]'; T2.t=[3,4];
obs1_scale=[3 0.5];  obs2_scale=[0.3 1];
obs1=obstacle2(T1,obs1_scale); obs2=obstacle2(T2,obs2_scale);

% ASAP solver 
asap=ASAP_problem(dim,range,{obs1,obs2});
asap.mapplot
% tracker plot
hold on 
plot(track_pos(1),track_pos(2),'ko') 

% parameters 
w_v=2; 

% graph 
G=graph();
G=G.addnode('t0');

% test example for cast ray from a position 
target=[5 6]';
track_pos=[1 4];

max_ray_length=3; % maximum length of ray to be checked 
N_azim=20; % how many rays are sampled?
N_stride=20; % stride number per one ray to check the occlusion
cast_result=asap.cast_ray(target,max_ray_length,N_azim,N_stride); % full cast ray result

% check the cast result 
hold on
plot(target(1),target(2),'ko','LineWidth',3);
idx_azim=1;
for azim=linspace(0,2*pi,N_azim)  % 2pi 
    plot([target(1) target(1)+cast_result(idx_azim)*cos(azim)],...
        [target(2) target(2)+cast_result(idx_azim)*sin(azim)],'r-','LineWidth',0.5);    
    idx_azim=idx_azim+1;
end


%% periodic SEDT on cast result

sample_ray_length=[3,2.5,2]; 
for i=1:length(sample_ray_length)
    cut_ray_length=sample_ray_length(i); % this is the cut distance before calculation of P-SEDT
    cast_result_binary=cast_result<cut_ray_length-0.1;
    periodic_cast_result_binary=repmat(cast_result_binary,1,3);
    D=signed_distance_transform(periodic_cast_result_binary);
    D=D(N_azim+1:2*N_azim); 

    % clustering 
    idx_nz_D=find(D~=0); %take nonzeros elements from D 
    cluster_res=DBSCAN(idx_nz_D',1,1);

    % consider the periodicity 
    if (D(1)~=0 && D(end)~=0)
        % merge the cluster 
        cluster_idx_rightend=cluster_res(find(idx_nz_D==N_azim));
        cluster_idx_leftend=cluster_res(find(idx_nz_D==1));    
        cluster_res(find(cluster_res==cluster_idx_rightend))=cluster_idx_leftend;    
    end

    cluster_indices=unique(cluster_res); % what indices are in the cluster res after merging? 
    cluster_candids=cell(length(cluster_indices),1);

    % for each cluster, find the 30% biggest rays 

    for insert_idx=1:length(cluster_indices)
        cur_cluster_index=cluster_indices(insert_idx);
        idx_idx_D_cur_cluster=find(cluster_res==cur_cluster_index);
        idx_D_cur_cluster=idx_nz_D(idx_idx_D_cur_cluster);
        D_cur_cluster=D(idx_D_cur_cluster);
        N_cur_cluster=length(D_cur_cluster);
        [~,sorted_idx] = sort(D_cur_cluster,'descend');
        save_idx=sorted_idx(1:ceil(N_cur_cluster*0.3));
        cluster_candids{insert_idx}=idx_D_cur_cluster(save_idx);        
    end

    
    %% graph construction from t0 of t1 

    for cluster_idx=1:length(cluster_candids) % iterate through cluster 
        candids=cluster_candids{cluster_idx}; % candidate points of this cluster
        n_candid=length(candids); 
        for t1_insertion_idx=1:n_candid
            t1_ray_idx=candids(t1_insertion_idx);
            ray_visibility=D(t1_ray_idx);
            ray_end=ray_ends(t1_insertion_idx,:);
            G=G.addnode(strcat( 't1/d= ',num2str(cut_ray_length),' azim_idx=',num2str(t1_ray_idx)));            
            G=G.addedge(strcat('t1/d=',num2str(cut_ray_length),' azim_idx=',num2str(t1_ray_idx)),'t0',norm(track_pos-ray_end)-w_v*ray_visibility/max(D));
        end
    end
    
end














