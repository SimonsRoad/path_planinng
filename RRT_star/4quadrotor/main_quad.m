% 연구노트 2권 p1 
%% obstacle class test
obs=obstacle(SE3(rotx(pi/6),ones(3,1)),[1 1 1]);
obs.plot
obs.isobs(5*ones(3,1))
hold on
axis equal
axis([-5 5 -5 5 -5 5])
xlabel('x'); ylabel('y'); zlabel('z');

%% Sampling test 
%%%%%%%%%%%%%
% MAP SETTING
%%%%%%%%%%%%%
dim=14; 
% map setting 
map_scale=[20 40 20]';
ws_range=[zeros(3,1) map_scale.*ones(3,1)]; % xyz scale of map
ws_range=[ws_range ; [zeros(3,1) [pi/6 pi/6 pi/2]']]; % angles of alpha and beta (see )
ws_range=[ws_range; repmat([0 pi/3 ; 0 2*pi],4,1)]; % angles of alpha and beta (see )
figure
offset=5;
axis([-offset map_scale(1)+offset 0-offset map_scale(2)+offset 0-offset map_scale(3)+offset]);
xlabel('x'); ylabel('y'); zlabel('z');
% obstacle setting
obs=obstacle(SE3(eye(3),[10 20 15]),[10 2 5]);
hold on
obs.plot()
draw_box(zeros(3,1),map_scale);
DrawAxis(SE3(eye(4)),5);
%% RRT*
quadprob=problem(dim,ws_range,{obs},8,1,2,5);
g=PGraph(dim);
g.add_node(x0);
g.set_gamma(1);
x0=[10 5 15 zeros(1,11)]';
xf=[10 30 15 zeros(1,11)]';
quadprob.PlotState(x0);
g_RRT=RRT_star(quadprob,g,500);
%%
p=g_RRT.Astar(1,g.closest(xf));

for v=p    
    hold on

    plot3(x0(1),x0(2),x0(3),'r*')
    plot3(xf(1),xf(2),xf(3),'r*')

    quadprob.PlotState(g_RRT.vertexlist(:,v))
    pause(1)
    
    
end





