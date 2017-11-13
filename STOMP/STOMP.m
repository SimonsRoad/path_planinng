%% sensor reading


real_map=robotics.BinaryOccupancyGrid(10,10,20);

obs_x=5; dx=1;
obs_y=5; dy=1;
Nx=200; Ny=200;

[xs,ys]=meshgrid(linspace(obs_x-dx,obs_x+dx,Nx),linspace(obs_y-dy,obs_y+dy,Ny));

obstacle=[reshape(xs,[],1) reshape(ys,[],1)];
setOccupancy(real_map,obstacle,1)


pose = [3,3,0];
ranges = 3*ones(100, 1);
angles = linspace(-pi/2+pi/4, pi/2+pi/4, 100);
raycast(real_map)

real_map.show()

%%
pose = [3,3,0];
ranges = 3*ones(100, 1);
angles = linspace(-pi/2, pi/2, 100);
maxrange = 20;
insertRay(map,pose,ranges,angles,maxrange);
show(map)



