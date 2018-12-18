function out=obstacle(t)
global obs1_path t_scale
t_data=(0:200)'*t_scale;
out=interp1(t_data,[obs1_path(1,:) ;obs1_path],t);

end
