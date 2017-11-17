function [res,occval]=cost_occupancy(xs,ys,occu_map)
    % occu_val is raw occupancy data along a trajectory 
    
   if iswithin([xs;ys],[occu_map.XWorldLimits;occu_map.YWorldLimits])
        occval=[];
        for i=1:length(xs)-1
            [endPts,midPts] = occu_map.raycast([xs(i) ys(i)],[xs(i+1) ys(i+1)]);
            occval=[occval ;occu_map.getOccupancy([midPts;endPts],'grid')];
        end

        occval(occval<0.3)=0;
        how_many_cross=sum(occval>0.6);
        cost1=5*traj_length(xs,ys); cost2=sum(occval.^3); cost3=1000*how_many_cross;
        res=cost1+cost2+cost3;
%         fprintf('trajectory cost : %f / uncertainty cost : %f / obstacle cost: %f \n',cost1,cost2,cost3);
   else
        occval=[];
        res=traj_length(xs,ys)^2+1000;  % path out of range
   end
   
   
   
   
end