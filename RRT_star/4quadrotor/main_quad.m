% 연구노트 2권 p1 
%% obstacle class test
obs=obstacle(SE3(rotx(pi/6),ones(3,1)),[1 1 1]);
obs.plot
obs.isobs(5*ones(3,1))
hold on
axis equal
axis([-5 5 -5 5 -5 5])
xlabel('x'); ylabel('y'); zlabel('z');

%%

