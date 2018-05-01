% this main file test the performance of polyfit functiona as Qv
addpath('C:\Users\junbs\Documents\path_planinng\image_tracking\cost');
addpath('C:\Users\junbs\Documents\path_planinng\common_util\')

%% import cast Ray result 
global p g w_d w_v elev_min elev_max N_azim N_elev castRayResultBinary coeff_surf d_track
load binary_example.mat castRayResultBinary
% SEDT

%% probelm setting 
elev_min=pi/6; elev_max=pi/4; d_track=4;
% N_azim , N_elev
N_elev=size(castRayResultBinary,1);
N_azim=size(castRayResultBinary,2);
[x,y]=meshgrid(linspace(0,2*pi,N_azim),linspace(elev_min,elev_max,N_elev));

p=[-3 0 1]'; g=[0 0 0]';
w_d=5; % weight for tracking distance 
w_v=2; % weight for visibility 
d_track=4;

elev_cur=atan2(p(3)-g(3),norm(p(1:2)-g(1:2)));
azim_cur=atan2(p(2)-g(2),p(1)-g(1))+pi/3;
azim_cur=pi;
if azim_cur<0
   azim_cur=azim_cur+2*pi; 
end

% obtain distant field around cur_azim 
D=signed_distance_transform(periodic_reshape(castRayResultBinary,azim_cur));
D=periodic_reshape(D,azim_cur);

% polynomial fitting cost
x=reshape(x,[],1)+(azim_cur-pi);
y=reshape(y,[],1);
z=reshape(-D,[],1);

coeff_surf=poly_surf_fit(x,y,z,5,3);
% fitting check
x0=[norm(p-g) azim_cur elev_cur]';
cost_fun_surf([0+(azim_cur-pi) 2*pi+(azim_cur-pi)],[elev_min elev_max],@(x) poly_surf_val(x,coeff_surf,5,3));
title('visibility cost')


%% cost ? 
sf=fit([x y],reshape(  mat,[],1),'cubicinterp')
plot(sf)
xlabel('azim')
ylabel('elev')
zlabel('d')
title('observability hard constraint')


%% optimization of total cost 
global history
options = optimoptions(@fmincon,'OutputFcn',@optim_logging_fun);
cost_fun_surf([0+(azim_cur-pi) 2*pi+(azim_cur-pi)],[elev_min elev_max],@(x) obj_fun_SEDT(x));
title('total object function')
x_sol=fmincon(@(x) obj_fun_SEDT(x),x0,[],[],[],[],[0 azim_cur-pi elev_min],[inf azim_cur+pi elev_max],[],options);

hold on
plot(history.x(1,2),history.x(1,3),'ro')
plot(history.x(:,2),history.x(:,3),'k*','LineWidth',3)
plot(history.x(end,2),history.x(end,3),'r*')





