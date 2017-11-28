function [res,occval,obs_where]=cost_occupancy(xs,ys,occu_map,varargin)
    % occu_val is raw occupancy data along a trajectory 
    flag=true;
   if iswithin([xs;ys],[occu_map.XWorldLimits;occu_map.YWorldLimits])
        occval=[];
        obs_where=[];
        for i=1:length(xs)-1
            [endPts,midPts] = occu_map.raycast([xs(i) ys(i)],[xs(i+1) ys(i+1)]);
            
            occval=[occval ;occu_map.getOccupancy([midPts;endPts],'grid')];
            
            
            if(sum(occval>0.7) && flag)
                obs_where=i;
                flag=false;
            end
            
        end
% 
        occval(occval<0.05)=0.02;
        how_many_cross=sum(occval>0.6);

         
        w2=100; w3=1000;  
        cost2=w2*prod(occval)^(1/length(occval));
        cost3=w3*how_many_cross;
        res=cost2(end)+cost3;
%         fprintf('trajectory cost : %f / uncertainty cost : %f / obstacle cost: %f \n',cost1,cost2,cost3);
   else
        occval=[];
        res=traj_length(xs,ys)^2+10000;  % path out of range
   end
   
   %this is for smoothing reconnection
   if length(varargin{1})>0
       w4=1000;
       v0=cell2mat(varargin{1});
       v1=[xs(2) ys(2)]-[xs(1) ys(1)];
       
       cos_theta=abs(dot(v0,v1)/(norm(v0)*norm(v1)));
       
       res=res+w4*(cos_theta-1)^4;
       
   end
   
   
end