function c1=assign_cent_single(x1,v1,centroids)
    % centroids = N x 2
    % we pick the exploration point which is the closet and velocity
    % direction 
    dist = norm(x1-centroids);
    w = 2.5;
    
    
    bearing_vec=(centroids - x1)/dist;
    vel_vec = v1/norm(v1);
    
    diff = norm(vel_vec - bearing_vec)^2;
    
    
    [~,min_idx] =min( w*diff + dist );
    fprintf("[%f , %f]\n",w*diff,dist);
    
    
    c1 = centroids(min_idx,:);
    
   

end