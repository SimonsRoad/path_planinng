% this is implmentation of CBBA algorithm (single task assignment to a fleet of unit)
% code was written by Jeon Bo Seong of Seoul National University of South
% Korea

%% problem definition 

Nt=4; Nu=3;

% target position (task)
g1=[2 4]; g2=[3 3 ]; g3=[4 3]; g4=[5 2]; gs=[g1;g2;g3;g4];
p1=[1 2]; p2=[2 1]; p3=[3 1]; ps=[p1;p2;p3];

gps=[gs ; ps];
% draw the problem 

figure
hold on 
for i=1:length(gs)
    plot(gs(i,1),gs(i,2),'r*')
end
    
for i=1:length(ps)
    plot(ps(i,1),ps(i,2),'bo')
end

axis( [min(gps(:,1))-0.5 max(gps(:,1))+0.5 min(gps(:,2))-0.5 max(gps(:,2))+0.5 ])

% matrix 
X=zeros(Nu,Nt);
Y=zeros(Nu,Nt);
Z=zeros(Nu,Nt);

B={};

%% 
for i=1:Nu
    
    
