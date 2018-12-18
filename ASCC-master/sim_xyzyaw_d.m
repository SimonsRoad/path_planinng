function state_d=xyzyaw_d(in)
t=in(1);
yaw_add=in(2);

global xd yd zd yaw t_scale
t_data=(0:200)'*t_scale;
x=interp1(t_data,[xd(1,2);xd(:,2)],t);
y=interp1(t_data,[yd(1,2);yd(:,2)],t);
z=interp1(t_data,[zd(1,2) ;zd(:,2)],t);
yaw_d=interp1(t_data,[yaw(1) yaw],t)+yaw_add;

state_d=double([x, y, z ,yaw_d]);

end

