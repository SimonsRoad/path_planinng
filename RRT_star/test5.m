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
e=0.3;
dmin=100;
isconnected=0;
while ~isconnected
g2.extend(prob,g1.extend(prob));
g1.extend(prob,g2.extend(prob));

g1.plot
g2.plot

for i=1:g1.n
   node_of_g1=g1.vertexlist(:,i);
   for j=g2.n
       node_of_g2=g2.vertexlist(:,j);
       if g1.distance_metric(node_of_g1,node_of_g2)<dmin
           dmin=g1.distance_metric(node_of_g1,node_of_g2);
       end
   end
    
end
dmin

isconnected=dmin<e;
hold on

pause(1e-1/2)

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

g3=PGraph(6,'distance','rpy','dweight',[1 1]);
g3.add_node(g1.vertexlist); g3.add_edge(g1.edgelist(1,:),g1.edgelist(2,:));
g3.add_node(g2.vertexlist); 
added_edge_list=g2.edgelist+n1;
g3.add_edge(added_edge_list(1,:),added_edge_list(2,:))
%%

for i=1:n1
   node_of_g1=g3.vertexlist(:,i);
   for j=1:n2
       node_of_g2=g3.vertexlist(:,j+n1);
       if g3.distance_metric(node_of_g1,node_of_g2)<e
           g3.add_edge(i,j+n1);
        
       end
   end
    
end
%%
figure()
prob.mapplot
hold on
plot3(root(1),root(2),root(3),'go','MarkerSize',5)
plot3(goal(1),goal(2),goal(3),'ro','MarkerSize',5)
xlabel('x')
ylabel('y')
zlabel('z')
grid on

draw_drone(root,1.5,0.5,2)
draw_drone(goal,1.5,0.5,2)


%%
p=g3.Astar(1,g3.closest(goal));

for v=p

    
    
    prob.mapplot
    hold on

    plot3(root(1),root(2),root(3),'r*')
    plot3(goal(1),goal(2),goal(3),'r*')

    draw_drone(g3.vertexlist(:,v),1.5,0.5,2)
    pause(1)
    
    
end





