function frontier_xy_near=get_frontier(occu_map,obs_center,obs_rad)

% occu_map : occupancy map class
% obs_center : center of observation
% obs_rad : radius of observation 


frontier_occu_map=occu_map.occupancyMatrix;
frontier_occu_map(frontier_occu_map<0.2)=0;
loc_idx1=frontier_occu_map>=0.2;
loc_idx2=frontier_occu_map<0.7;
loc_idx=logical (loc_idx1 .* loc_idx2);
frontier_occu_map(loc_idx)=0.5;
frontier_occu_map(frontier_occu_map>0.7)=2;

pad=zeros(occu_map.GridSize(1)+2,occu_map.GridSize(2)+2);
mask=pad;
mask(2:end-1,2:end-1)=frontier_occu_map;

conv1=pad;
conv1(1:end-2,2:end-1)=frontier_occu_map;

res=abs(mask-conv1);
[row,col]=find(res==0.5);

frontier_idx=[row col];


conv1=pad;
conv1(2:end-1,1:end-2)=frontier_occu_map;

res=abs(mask-conv1);
[row,col]=find(res==0.5);

frontier_idx=[frontier_idx;[row col]];

conv1=pad;
conv1(3:end,2:end-1)=frontier_occu_map;

res=abs(mask-conv1);
[row,col]=find(res==0.5);

frontier_idx=[frontier_idx;[row col]];


conv1=pad;
conv1(2:end-1,3:end)=frontier_occu_map;

res=abs(mask-conv1);
[row,col]=find(res==0.5);

frontier_idx=[frontier_idx;[row col]];


erase_idx=[];

for i=1:length(frontier_idx)
    
    if (sum(frontier_idx(i,1)==[1 2 occu_map.GridSize(1)+1 occu_map.GridSize(1)+2])>0) || (sum(frontier_idx(i,2)==[1 2 occu_map.GridSize(2)+1 occu_map.GridSize(2)+2])>0) 
        erase_idx=[erase_idx i];
    end
    
end


frontier_idx(erase_idx,:)=[];
frontier_idx=frontier_idx-ones(length(frontier_idx),2);


res=zeros(200);
for i=1:length(frontier_idx)
    res(frontier_idx(i,1),frontier_idx(i,2))=1;
end

frontier_xy=occu_map.grid2world(frontier_idx);

% so far, we obtained all the frontier cells in the map
% now let's select the cells around an observation center and radius

dists=sqrt(sum((frontier_xy-obs_center).^2,2));
frontier_near_idx=(dists<obs_rad);

frontier_xy_near=frontier_xy(frontier_near_idx,:);

end



