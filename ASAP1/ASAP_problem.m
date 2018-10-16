classdef ASAP_problem < handle
    properties 
        dim  % dimension of problem (normally 2)
        ws_range % workspace /size: D x 2 
        obs_list % list of obstacle 
        G % graph
        w_v % visibility weight 
        layer_info_num % vector number of nodes in each layer 
        layer_info_pose % cell of positions of nodes in each layer 
        tracker_pos % current tracker position 
        N_stride  % stride number per one ray to check the occlusion
        N_azim % how many rays are sampled?
        sample_ray_length; 
        max_ray_length; % maximum length of ray to be checked     
        azim_set; % a bit redundant, but usefel 
        PM; % this path manager will generate traj connecting each view points also avoid obstacles 
    end
    
    methods
        function obj=ASAP_problem(dim,range,obs_list,w_v,N_stride,N_azim,max_ray_length,sample_ray_length,path_param)
        obj.dim=dim;
        obj.ws_range=range;
        obj.obs_list=obs_list;
        obj.G=digraph();
        %obj.G=graph(); % empty graph 
        obj.w_v=w_v;
        obj.layer_info_num=[];
        obj.layer_info_pose={};
        obj.tracker_pos=[]; % this will be initialized when graph_init is instantiated 
        obj.N_stride=N_stride;
        obj.N_azim=N_azim;
        obj.max_ray_length=max_ray_length;
        obj.sample_ray_length=sample_ray_length; 
        obj.azim_set=linspace(0,2*pi,N_azim);
        obj.PM=path_manager(obs_list,path_param);        
        end
        
        %% High level methods
        % this function construct layer info 
        function [layer_nodes,node_pos,node_vis]=layer_calc(ASAP_problem,target,t_idx)
            
            % input : light _source ([1 x 2]) / t_idx (what sequence is it now?)
            % output : layer_nodes (cell of names) / node_pos (cell of
            % position of the nodes) /node_vis (node visibililty )
            
            layer_nodes={}; 
            node_pos={};
            node_vis=[];
            n_node=0;
            node_insertion_idx=1;
            
            
            cast_result=ASAP_problem.cast_ray(target); % full cast ray result
            
            for i=1:length(ASAP_problem.sample_ray_length) % we construct a layer by adding nodes from different cut length 
                
                % step1 : find P-SEDT of binary cast ray result
                cut_ray_length=ASAP_problem.sample_ray_length(i); % this is the cut distance before calculation of P-SEDT
                cast_result_binary=cast_result<cut_ray_length;
                periodic_cast_result_binary=repmat(cast_result_binary,1,3);
                D=signed_distance_transform(periodic_cast_result_binary);
                D=D(ASAP_problem.N_azim+1:2*ASAP_problem.N_azim); 
                
                % step 2 clustering 
                idx_nz_D=find(D~=0); %take nonzeros elements from D 
                cluster_res=DBSCAN(idx_nz_D',1,1);

                % consider the periodicity 
                if (D(1)~=0 && D(end)~=0)
                    % merge the cluster 
                    cluster_idx_rightend=cluster_res(find(idx_nz_D==ASAP_problem.N_azim));
                    cluster_idx_leftend=cluster_res(find(idx_nz_D==1));    
                    cluster_res(find(cluster_res==cluster_idx_rightend))=cluster_idx_leftend;    
                end

                cluster_indices=unique(cluster_res); % what indices are in the cluster res after merging? 
                cluster_candids=cell(length(cluster_indices),1);

                % step 3: obtain the candidates of nodes 

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
                
                % step 4: add the nodes to layer 
                 
                 for cluster_idx=1:length(cluster_candids) % iterate through cluster 
                    candids=cluster_candids{cluster_idx}; % candidate points of this cluster
                    n_candid=length(candids); 
                    for insertion_idx=1:n_candid
                        ray_idx=candids(insertion_idx); % this is the name of this node 
                        end_pnt=ASAP_problem.ray_ends(target,cut_ray_length,ASAP_problem.azim_set(ray_idx));                        
                         n_node=n_node+1;

                        % name of this node in the layer 
                        layer_nodes{node_insertion_idx}=strcat('t',num2str(t_idx),'/',num2str(n_node));
                        node_pos{node_insertion_idx}=end_pnt;
                        node_vis(node_insertion_idx)=D(ray_idx);
                        
                        node_insertion_idx=node_insertion_idx+1;
                        
                    end
                 end
                                
            end   % for loop for cut length                              
        end
        
        function path=path_proposal(ASAP_problem)
            path_idx_full=ASAP_problem.G.shortestpath('t0','tf','Method','auto');
            path_idx={};
            for j=1:length(path_idx_full)-2
                path_idx{j}=path_idx_full{j+1};
            end
                       
            path=[];
            
            for i=1:length(path_idx)
                % parsing 
                seq=str2num(path_idx{i}(2)); key=str2num(path_idx{i}(4:end));
                path=[path ; ASAP_problem.layer_info_pose{seq}{key}];                
            end
            
            
        end
              
        function graph_init(ASAP_problem,cur_tracker_pos)
            % the graph will be constructed with the cur_tracker_pose as
            % t0 points     
            ASAP_problem.G=digraph();
            cur_tracker_pos=reshape(cur_tracker_pos,1,2);
            ASAP_problem.tracker_pos=cur_tracker_pos;
            ASAP_problem.layer_info_num=[];  
            ASAP_problem.layer_info_pose=[];  
            ASAP_problem.G =   ASAP_problem.G.addnode('t0');            
        end
        
        % this function ends the graph construction
        function graph_wrapper(ASAP_problem)
                ASAP_problem.G=ASAP_problem.G.addnode('tf');
                num_node_last=ASAP_problem.layer_info_num(end);
                pose_node_last=ASAP_problem.layer_info_pose{end};
                num_layer=length(ASAP_problem.layer_info_num);
                for connect_idx=1:num_node_last
                    node_name=strcat('t',num2str(num_layer),'/',num2str(connect_idx));
                    ASAP_problem.G=ASAP_problem.G.addedge(node_name,'tf',0.1);
                end                    
        end
                
        % this function insert a layer to graph
        function graph_insertion(ASAP_problem,target)
            if isempty(ASAP_problem.layer_info_num)  % we need to connect with  t0 
                t_idx=1;
                
                [layer_names,layer_poses,layer_vises]=ASAP_problem.layer_calc(target,t_idx);
                n_nodes=length(layer_names);
                ASAP_problem.layer_info_num=[ASAP_problem.layer_info_num n_nodes];
                ASAP_problem.layer_info_pose{t_idx}=layer_poses; % we inserted cell to cell 
                
                % connect nodes of this layer with tracker position 
                for connect_idx=1:n_nodes
                    name_node=strcat('t',num2str(t_idx),'/',num2str(connect_idx));
                    ASAP_problem.G=ASAP_problem.G.addnode(name_node);
                    dist=norm(ASAP_problem.tracker_pos-layer_poses{connect_idx});
                    vis=layer_vises(connect_idx)/max(layer_vises);
                    
                    if isnan(vis) || (vis==0)
                        weight=dist^2;
                    else
                        weight=dist^2+(ASAP_problem.w_v)/vis;
                    end
                    
                    if dist<4
                     ASAP_problem.G=ASAP_problem.G.addedge('t0',name_node,weight);
                    end
                end %connect 
            
            else % already exsit a layer
                t_idx=length(ASAP_problem.layer_info_num)+1; % insert idx at this time
                [layer_names,layer_poses,layer_vises]=ASAP_problem.layer_calc(target,t_idx); % layer calculation
                n_nodes=length(layer_names);
                
                % add layer to nodes of graph 
                 for add_idx=1:n_nodes
                        ASAP_problem.G=ASAP_problem.G.addnode(layer_names{add_idx});
                  end 
                                
                % update layer information
                ASAP_problem.layer_info_num=[ASAP_problem.layer_info_num n_nodes];
                ASAP_problem.layer_info_pose{t_idx}=layer_poses;
                
                % connect with previous layer 
                prev_layer_poses=ASAP_problem.layer_info_pose{t_idx-1};               
                cur_layer_poses=ASAP_problem.layer_info_pose{t_idx};                
                for prev_idx=1: ASAP_problem.layer_info_num(t_idx-1)
                    for idx=1:ASAP_problem.layer_info_num(t_idx)
                        prev_pose=prev_layer_poses{prev_idx};  cur_pose=cur_layer_poses{idx};
                        prev_name=strcat('t',num2str(t_idx-1),'/',num2str(prev_idx));
                        dist=norm(prev_pose-cur_pose);   vis=layer_vises(idx)/max(layer_vises);
                        if isnan(vis) || (vis==0)
                            weight=dist^2;
                        else
                            weight=dist^2+(ASAP_problem.w_v)/vis;
                        end
                        if dist <6
                         ASAP_problem.G=ASAP_problem.G.addedge(prev_name,layer_names{idx},weight);                        
                        end
                    end                    
                end
                          
            end % inserted into graph
                        
            
        end
        
        %% low level methods 
       
        function end_pnts=ray_ends(ASAP_problem,light_source,ray_ds,ray_ang)
            % this function cacluate the endpoints (R2)
            % inputs: light_source (1 x 2) / lengths of rays (N x 1) /
            % ray_anges (N x 1)
            % output: end_pnts (N x 2)
            
            num_query=length(ray_ds);
            light_source=reshape(light_source,1,2);
            end_pnts=zeros(num_query,2); 
            for query=1:num_query
                end_pnts(query,:)=light_source+[ray_ds(query)*cos(ray_ang(query)) ray_ds(query)*sin(ray_ang(query))];
            end
        end
        
        function mapplot(ASAP_problem)
            % this funnction plot the current problem settings
            hold on 
            axis([ASAP_problem.ws_range(1,1) ASAP_problem.ws_range(1,2)...
                ASAP_problem.ws_range(2,1) ASAP_problem.ws_range(2,2)])
                       
            for i=1:length(ASAP_problem.obs_list)
                obs=ASAP_problem.obs_list{i};
                obs.plot
            end
            
            if ~isempty(ASAP_problem.tracker_pos)
                plot(ASAP_problem.tracker_pos(1),ASAP_problem.tracker_pos(2),'ko')
            end
            hold off
        end
        
        function cast_result=cast_ray(ASAP_problem,light_source)
            % this funciton cast N rays from the given target position 
            % input : light source ( D x 1) / max_distance () /
            % N_azim : 2pi / N_stride : check points of a ray           
            % output : 1 x N hit distance result 
            N_azim=ASAP_problem.N_azim;
            N_stride=ASAP_problem.N_stride;
            max_distance=ASAP_problem.max_ray_length;
            cast_result=zeros(1,N_azim);
            light_source=reshape(light_source,1,ASAP_problem.dim);
            azim_idx=1;
            for azim=linspace(0,2*pi,N_azim)  % 2pi 
                light_dir=[cos(azim) sin(azim)];
                for  stride_len=linspace(0,max_distance,N_stride) % for stride of an ray 
                    check_point=light_source+light_dir*stride_len; % we inspect whether this points is in the obstacle region
                    
                    for obs_idx=1:length(ASAP_problem.obs_list) % for obstacle list
                        this_obs=ASAP_problem.obs_list{obs_idx};
                        if this_obs.isobs(check_point')
                            break
                        end                        
                    end  % for all obstacle
                    
                        if this_obs.isobs(check_point')  % if this stride is hit
                            break
                        end
                        
                end                
                cast_result(azim_idx)=stride_len;
                azim_idx=azim_idx+1;
            end % 2pi 
            
        end % function end 
         
        
    end
    
end