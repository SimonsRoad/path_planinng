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
map_scale=[20 20 20]';
ws_range=[zeros(3,1) map_scale.*ones(3,1)]; % xyz scale of map
ws_range=[ws_range ; [zeros(3,1) [pi/3 pi/3 pi/2]']];
ws_range=[ws_range; repmat([0 pi/3 ; 0 2*pi],4,1)]; % angles of alpha and beta (see )
figure
offset=5;
axis([-offset map_scale(1)+offset 0-offset map_scale(2)+offset 0-offset map_scale(3)+offset]);
xlabel('x'); ylabel('y'); zlabel('z');
% obstacle setting
obs=obstacle(SE3(eye(3),[10 10 3]),[5 3 3]);
hold on
obs.plot()
%% sampling 
quadprob=problem(dim,ws_range,{obs},8,1,2,3);
sampled_state=quadprob.sample;
quadprob.PlotState(sampled_state);

hold on


