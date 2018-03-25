function value=potential_field(query_x,repulsive_centers,repulsive_cluster_numel,safety_distance)
    % query_x : D x 1
    % repulsive centers : N_cluster x D
    % repulsive cluster numel : N_cluster x 1
    % safty_distance 
    query_x=reshape(query_x,1,length(query_x));
    coeffi=100;
    value=0;
    for  cluster_idx=1:length(repulsive_cluster_numel)
        
        dist=norm(query_x-repulsive_centers(cluster_idx,:));
        if dist  < safety_distance
            value=0.5*coeffi*repulsive_cluster_numel(cluster_idx)*(1/dist-1/safety_distance)^2;
        end        
    end
   
end