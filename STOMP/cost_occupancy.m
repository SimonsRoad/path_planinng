function [res,occval]=cost_occupancy(xs,ys,occu_map)
    % occu_val is raw occupancy data along a trajectory 
    
   if iswithin([xs;ys],[occu_map.XWorldLimits;occu_map.YWorldLimits])
    
        occval=[];
        for i=1:length(xs)-1
            [endPts,midPts] = occu_map.raycast([xs(i) ys(i)],[xs(i+1) ys(i+1)]);
            occval=[occval ;occu_map.getOccupancy([midPts;endPts],'grid')];
        end

        occval(occval<0.3)=0;
        res=traj_length(xs,ys)^2 +10*sum(occval.^2)/traj_length(xs,ys);
   else
        occval=[];
        res=traj_length(xs,ys)^2+1000;  % path out of range
   end
end