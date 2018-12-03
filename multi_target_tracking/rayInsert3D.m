function res = rayInsert3D(map3,ray_origin,azim,elev,ray_length,check_res)
% this function is for ray insertion 
    
% map3 : OccupancyMap3D object in robotics tool box 
% check_res : parameter for stride of check pnt 
% res = 1 if hit or 0 otherwise    
   ray_origin = reshape(ray_origin,1,[]);
   
    for l = 0:check_res:ray_length 
        pnt = ray_origin + l*[cos(elev)*cos(azim) cos(elev)*sin(azim) sin(elev)];
        if map3.checkOccupancy(pnt) == 1% if unknown : -1  / occupied 1 
            res = 1;
            return;
        end
    end    
    res = 0;  

    
     
end