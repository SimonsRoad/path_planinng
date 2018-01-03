function rayinsertion(pose,angle_min,angle_max,Nray,maxrange)

global real_map 
global occu_map 

angles = linspace(angle_min, angle_max,Nray);
ranges=maxrange*ones(1,Nray);


for i=1:Nray
    intPnt=real_map.rayIntersection(pose,angles(i),maxrange);
    if ~isnan(intPnt(1)) % if hits any obstacle
        ranges(i)=norm(pose(1:2)-intPnt);
    end
    
end


startpt=pose(1:2);

occu_map.insertRay(pose,ranges,angles,maxrange,[0 0.8])






