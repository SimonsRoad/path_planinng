%% Demo env

origin=[0 0];
lx=10; ly=10; Nx=100; Ny=100; dx=(lx/Nx); dy=(ly/Ny);
occumap=gridmap(origin,dx,dy,Nx,Ny);

occumap.obstacle_assign([7,4],[1,2],0.9)
occumap.obstacle_assign([7,1],[1,1],0.4)
occumap.mapplot

%% path generation using RRT(*)



% plot(xs,ys,'r','LIneWidth',2)
