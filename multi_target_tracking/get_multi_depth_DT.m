function [DT_set,pinch_bin] = get_multi_depth_DT(map3,ray_origin,azim_set,elev_set,max_ray,ray_cast_res,clustering_delta_r)
    % This function get a multi-layered distance field 
    % clustering epsilon : clustering_delta_r 
    % ray cast res : ray cast resolution (normally 1/map3.res)
    
    %% Phase 1 : investigate the hit distance toward every direction 
    N_azim = length(azim_set);
    N_elev = length(elev_set);
    
    hit_dist = zeros(N_azim,N_elev);
    
    for azim_idx = 1:N_azim
        for elev_idx = 1:N_elev
            hit_dist(azim_idx,elev_idx) = raycast3D(map3,ray_origin,azim_set(azim_idx),elev_set(elev_idx),max_ray,ray_cast_res);    % the hit distance into that direction        
        end
    end
    
   %% Phase 2: clustering the hit distance 
   
   hit_dist_flat=reshape(hit_dist,1,[]); % histogram
   % tuning!! 
   N_min_cluster = 20;
   IDX=DBSCAN(hit_dist_flat',clustering_delta_r,N_min_cluster);
   if max(IDX) == 1
    warning('clusterring warning : clustered altogether');
   end
   pinch_bin = [];
   
   % what are the representative dividing value of hit distance 
   for idx = 1:max(IDX)
        pinch_bin=[pinch_bin min(hit_dist_flat(find(IDX==idx)))-0.02];       
   end
   
   %% Phase 3: set DT    
   DT_set = {};
   pinch_bin = sort(pinch_bin);
   for i = 1:length(pinch_bin)
       pinch = pinch_bin(i);
       cast_res = double(hit_dist < pinch); 
       DT = signed_distance_transform([cast_res ; cast_res ; cast_res]); % periodic distance transform         
       DT = DT(N_azim+1:2*N_azim,:);  
       DT_set{i} = DT;
   end

end