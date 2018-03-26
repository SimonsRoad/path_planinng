% trajectory generation 
load('Data.mat');
Data=Data(:,3:end);


global nb dt
nb=170;
xd=Data(1,1:nb);
yd=Data(2,1:nb);


xd=linspace(-2,2,nb);
yd=linspace(-2,2,nb);
zd=linspace(-2,2,nb);



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
figure
title('xd')
subplot(3,1,1)
plot(xd)
subplot(3,1,2)
plot(smoothdata(xd_dot))
subplot(3,1,3)
plot(smoothdata(xd_ddot))

figure
title('yd')
subplot(3,1,1)
plot(yd)
subplot(3,1,2)
plot(smoothdata(yd_dot))
subplot(3,1,3)
plot(smoothdata(yd_ddot))

figure
title('zd')
subplot(3,1,1)
plot(zd)
subplot(3,1,2)
plot(zd_dot)
subplot(3,1,3)
plot(zd_ddot)







