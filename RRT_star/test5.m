g1=PGraph(6,'distance','rpy','dweight',[1 1]); g2=PGraph(6,'distance','rpy','dweight',[1 1]);
g1.set_gamma(3); g2.set_gamma(3);

ndim=3;
obs{1}=[0 2;3 15;-5 5];
obs{2}=[0 2;-15 -3;-5 5];
obs{3}=[0 2 ;-3 3;-5 -1];
obs{4}=[0 2;-3 3;1 5];

% for i=1:length(obs)
%     obs{i}=rotx(pi/6)*obs{i};
% end



ranges=[-10 10;-15 15 ; -5 5 ;-pi/3 pi/3; 0 pi/6 ;-pi/6 pi/6];
prob=problem(ndim,ranges,obs,3);



root=[-7 10 0 -pi/6 0 0]';
goal=[7 -10 0 0 0 0]';


g1.add_node(root); g2.add_node(goal);


%%

prob.mapplot
hold on
plot3(root(1),root(2),root(3),'go','MarkerSize',5)
plot3(goal(1),goal(2),goal(3),'ro','MarkerSize',5)
xlabel('x')
ylabel('y')
zlabel('z')
grid on


%%
for i=1:15
g2.extend(prob,g1.extend(prob));
g1.extend(prob,g2.extend(prob));

g1.plot
g2.plot
hold on

pause(1e-1)

end

%%
prob.mapplot
hold on
plot3(root(1),root(2),root(3),'go','MarkerSize',5)
plot3(goal(1),goal(2),goal(3),'ro','MarkerSize',5)
xlabel('x')
ylabel('y')
zlabel('z')
grid on

g1.plot
g2.plot
hold on



%% two graph merge 

n1=g1.n; n2=g2.n;

g3=g1;
g3.add_node(g2.vertexlist);
added_edge_list=g2.edgelist+n1;
g3.add_edge(added_edge_list(1,:),added_edge_list(2,:))


