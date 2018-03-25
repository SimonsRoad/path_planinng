function cost_fun_surf(x_bound,y_bound,cost_fun)
% this function plot the current cost(x,y) in boundary region 
costs=[];
N_azim_sample=100; N_elev_sample=60;
azim_sample=linspace(x_bound(1),x_bound(2),N_azim_sample);
elev_sample=linspace(y_bound(1),y_bound(2),N_elev_sample);

[azim_mesh,elev_mesh]=meshgrid(azim_sample,elev_sample);
azim_mesh=reshape(azim_mesh,[],1);
elev_mesh=reshape(elev_mesh,[],1);


for q_idx=1:length(azim_mesh)
    cur_cost=cost_fun([azim_mesh(q_idx) elev_mesh(q_idx)]);
    costs=[costs cur_cost];
end

figure()
title('cost_fun evaluation')
surf(reshape(azim_mesh,N_elev_sample,N_azim_sample),...
    reshape(elev_mesh,N_elev_sample,N_azim_sample),...
    reshape(costs,N_elev_sample,N_azim_sample))
xlabel('x'); ylabel('y')

end

