%% Demo env

origin=[0 0];
lx=10; ly=10; Nx=100; Ny=100; dx=(lx/Nx); dy=(ly/Ny);
occumap=gridmap(origin,dx,dy,Nx,Ny);

occumap.obstacle_assign([7,4],[1,2],0.91)
occumap.obstacle_assign([7,1],[1,1],0.4)
occumap.mapplot
hold on

%% path generation using RRT(*)

x0=[1 3 0]';
xf=[9 3 0]';
dim=3; % x(1:2)= xy position   x(3)=probability of obstacle 
g=PGraph(dim);
g.add_node(x0)

g=pRRT(occumap,g,3000,xf);
g.plot('NodeSize',2)

% plot(xs,ys,'r','LIneWidth',2)


%%

