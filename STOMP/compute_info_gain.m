function info_metric=compute_info_gain(xys,info_rad,unknown_threshold)
% this function compute how many cells are flaged as unkown around a point
% within given radius 
global occu_map
Nray=20;
info_metric=zeros(size(xys,1),1);
for i=1:size(xys,1)
    
    pose=[xys(i,:) 0];
    ranges=info_rad*ones(Nray,1);
    angles=linspace(0,2*pi,Nray);
    count=0;

    for n=1:Nray
        [~,midpoints]=occu_map.raycast(pose,ranges(n),angles(n));
        occval=occu_map.getOccupancy(midpoints,'grid');
        count=count+sum(occval>unknown_threshold);
    end
    
    info_metric(i)=count;
    
    
end
