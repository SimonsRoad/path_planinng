%% load initial data
load binary_example.mat castRayResultBinary
addpath('../')
addpath('C:\Users\junbs\Documents\path_planinng\common_util\')
%% method 1 normal signed distance transform 
D=signed_distance_transform(castRayResultBinary);
%  linear cost 
figure(1)
title('linear interpolation')
surf(-D)
elev_min=pi/6; elev_max=pi/4;
% axis equal
[x,y]=meshgrid(linspace(0,2*pi,size(castRayResultBinary,2)),linspace(elev_min,elev_max,size(castRayResultBinary,1)));

% polynomial fitting cost
x=reshape(x,[],1);
y=reshape(y,[],1);
z=reshape(-D,[],1);

coeff_self=poly_surf_fit(x,y,z,5,3);

figure(2)
title('polynomial fitting')
sf=fit([x y],z,'poly53');
p_fit=coeffvalues(sf);
plot(sf,[x,y],z)
% axis equal

 % TODO : check cost by plotting 
 cost_fun_surf([0 2*pi],[elev_min elev_max],@(x) fit_poly_fun(x,sf));
 cost_fun_surf([0 2*pi],[elev_min elev_max],@(x) poly_surf_val(x,coeff_self,5,3));
 
 

%% optimization performance for this cost only  ,                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
x0=[5 0.7]; %initial point 
global history
options = optimoptions(@fmincon,'OutputFcn',@optim_logging_fun);
x_sol=fmincon(@(x) fit_poly_fun(x,sf),x0,[],[],[],[],[0 elev_min],[2*pi elev_max],[],options);
figure(3)
plot(sf,[x,y],z)
hold on
plot(history.x(:,1),history.x(:,2),'b*')
plot(history.x(end,1),history.x(end,2),'r*')

%% method2 sum of seperate distance transform field (turned out to be bad)

%clustering 
[occlusion_idx_y,occlusion_idx_x]=find(castRayResultBinary==1);
% numel(occlusion_idx_x==idx_y)
cluster_idces=DBSCAN([occlusion_idx_x occlusion_idx_y],2,1); %
[N_row,N_col]=size(castRayResultBinary);
for cluster_idx=1:max(cluster_idces)
    mat= zeros(N_row,N_col); 
    this_cluster_occlusion_idx_x=occlusion_idx_x(cluster_idces== cluster_idx);
    this_cluster_occlusion_idx_y=occlusion_idx_y(cluster_idces== cluster_idx);     
    for i=1:length(this_cluster_occlusion_idx_x)        
        mat(this_cluster_occlusion_idx_y(i),this_cluster_occlusion_idx_x(i))=1;
    end
    cluster_occlusion_matrix{cluster_idx}=mat;
end

% cluster occlusion matrix = clustered occlusion matrix 
D1=signed_distance_transform(logical(cluster_occlusion_matrix{1}));
D2=signed_distance_transform(logical(cluster_occlusion_matrix{2}));
surf(-D1-D2)










