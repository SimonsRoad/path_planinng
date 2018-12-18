function out=xdot_ee(t)
global xdotl_repro t_scale

t_data=(0:200)'*t_scale;
out=interp1(t_data,[xdotl_repro(:,1) xdotl_repro]'*t_scale,t);

end
