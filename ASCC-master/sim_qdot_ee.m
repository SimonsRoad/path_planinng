function out=qdot_ee(t)
global qdot_DMP t_scale
t_data=(0:200)'*t_scale;
out=interp1(t_data,[qdot_DMP(:,1) qdot_DMP]'*t_scale,t);

end
