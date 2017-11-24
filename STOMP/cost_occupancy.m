function [res,occval]=cost_occupancy(xs,ys,occu_map)
    % occu_val is raw occupancy data along a trajectory 
   if iswithin([xs;ys],[occu_map.XWorldLimits;occu_map.YWorldLimits])
        occval=[];
        for i=1:length(xs)-1
            [endPts,midPts] = occu_map.raycast([xs(i) ys(i)],[xs(i+1) ys(i+1)]);
            occval=[occval ;occu_map.getOccupancy([midPts;endPts],'grid')];
        end
% 
        occval(occval<0.05)=0.05;
        how_many_cross=sum(occval>0.6);

%         w1=50;
%         cost1=w1*traj_length(xs,ys);

        w2=100; w3=1000;  
        cost2=w2*prod(occval)^(1/length(occval));
        cost3=w3*how_many_cross;
        res=cost2(end)+cost3;
%         fprintf('trajectory cost : %f / uncertainty cost : %f / obstacle cost: %f \n',cost1,cost2,cost3);
   else
        occval=[];
        res=traj_length(xs,ys)^2+10000;  % path out of range
   end
   
   
end