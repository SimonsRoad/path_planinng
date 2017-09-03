% trajectory generation 
load('Data.mat');
Data=Data(:,3:end);
global nb dt
nb=170;
xd=Data(1,1:nb);
yd=Data(2,1:nb);
zd=linspace(-1,1,nb);



rd=[xd ; yd; zd];
plot3(xd,yd,zd)

% velocity  
xd_dot=diff(xd)/dt;
yd_dot=diff(yd)/dt;
zd_dot=diff(zd)/dt;
rd_dot=[xd_dot ; yd_dot ;zd_dot];

% acceleration
xd_ddot=diff(xd_dot)/dt;
yd_ddot=diff(yd_dot)/dt;
zd_ddot=diff(zd_dot)/dt;

rd_ddot=[xd_ddot ; yd_ddot ;zd_ddot];

% yaw angle
yawd=atan3(yd_dot,xd_dot);









