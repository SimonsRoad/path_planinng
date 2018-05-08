%% spline generation 
% spline order 
n=5;
waypoint={};
x0=[0 0];
xdot0=[0 0];
waypoint{1}=[1,2];
waypoint{2}=[2 3];
w_v=3;
sp=spline_path(n,x0,xdot0,waypoint,w_v);


%% plot 
t=linspace(0,1,10);
x=zeros(length(t),1);
for i=1:length(t)
    t_=t(i);
    x(i)=p'*time_vector(t_,n,0);    
end
figure
plot(t,x,'b-')






