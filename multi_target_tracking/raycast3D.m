function hit_dist = raycast3D(map3,ray_origin,azim,elev,max_ray_length,ray_cast_res)
% this function cast a ray with maximum length 
    
% map3 : OccupancyMap3D object in robotics tool box 
% check_res : parameter for stride of check pnt 
% res = 1 if hit or 0 otherwise    
   ray_origin = reshape(ray_origin,1,[]);
   
    for l = 0:ray_cast_res:max_ray_length 
        pnt = ray_origin + l*[cos(elev)*cos(azim) cos(elev)*sin(azim) sin(elev)];
        if map3.checkOccupancy(pnt) == 1% if unknown : -1  / occupied 1 
            hit_dist = l;
            return;
        end
    end    
    hit_dist = max_ray_length;  
     
end