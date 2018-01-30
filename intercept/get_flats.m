function[traj] = get_flats(p,t)
%% p=[px ; py; pz]
poly_order=length(p)-1;
px=p(1,:); py=p(2,:); pz=p(3,:);

traj.x = [px*t_vector(t,0,poly_order);py*t_vector(t,0,poly_order); pz*t_vector(t,0,poly_order)];
traj.dx = [px*t_vector(t,1,poly_order);py*t_vector(t,1,poly_order); pz*t_vector(t,1,poly_order)];
traj.d2x = [px*t_vector(t,2,poly_order);py*t_vector(t,2,poly_order); pz*t_vector(t,2,poly_order)];
