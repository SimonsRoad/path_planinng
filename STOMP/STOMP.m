%% sensor reading test

% real map contruction 
real_map=robotics.OccupancyGrid(10,10,20);

obs_x=[3 7]; dx=[1 1];
obs_y=[5 3]; dy=[3 1];
Nx=200; Ny=200;
Nobs=length(obs_x);

for I =1: Nobs

    [xs,ys]=meshgrid(linspace(obs_x(I)-dx(I),obs_x(I)+dx(I),Nx),linspace(obs_y(I)-dy(I),obs_y(I)+dy(I),Ny));
    obstacle=[reshape(xs,[],1) reshape(ys,[],1)];
    setOccupancy(real_map,obstacle,1)
end

real_map.show()

pose = [1,1,0];
maxrange = 10;
Nray=500;
angles = linspace(-pi/2+pi/4, pi/2+pi/4, Nray);
ranges=maxrange*ones(1,Nray);

for i=1:Nray
intPnt=real_map.rayIntersection(pose,angles(i),maxrange);
    if ~isnan(intPnt(1)) % if hits any obstacle
        ranges(i)=norm(pose(1:2)-intPnt);
    end    
end

% occumap contruction 
occu_map=robotics.OccupancyGrid(10,10,20);
occu_map.insertRay(pose,ranges,angles,maxrange)
occu_map.insertRay(pose,ranges,angles,maxrange)
mat=occu_map.occupancyMatrix;
occu_map.show()


%% cost metric 

xs=[1 3 6 9];
ys=[1 3 5 9];

hold on 

plot(xs,ys,'r-')
cost=0;
occval=[];
for i=1:length(xs)-1
    [endPts,midPts] = occu_map.raycast([xs(i) ys(i)],[xs(i+1) ys(i+1)]);
    occval=[occval ;occu_map.getOccupancy([midPts;endPts],'grid')];
end






