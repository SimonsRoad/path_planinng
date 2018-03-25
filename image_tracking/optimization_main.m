%% data generation 
global castRayResult param

load example_raycast_result.mat mat
% default 
castRayResult=mat;
elev_min=pi/8; elev_max=pi/3;
param=[0 2*pi size(castRayResult,2) elev_min elev_max size(castRayResult,1) ];
[x,y]=meshgrid(linspace(0,2*pi,size(castRayResult,2)),linspace(elev_min,elev_max,size(castRayResult,1)));
z=castRayResult;
x=reshape(x,[],1);
y=reshape(y,[],1);
z=reshape(z,[],1);

% clustering 
castRayResultBinary=mat<4;

[occlusion_idx_y,occlusion_idx_x]=find(castRayResultBinary==1);
% numel(occlusion_idx_x==idx_y)
azim_set=linspace(0,2*pi,size(castRayResult,2));
elev_set=linspace(elev_min,elev_max,size(castRayResult,1));

azim_occlusion=azim_set(occlusion_idx_x);
elev_occlusion=elev_set(occlusion_idx_y);

cluster_idx=DBSCAN([occlusion_idx_x occlusion_idx_y],2,1); %

occlusion_cluster_centers=[]; %N_cluster x 2 
occlusion_cluster_numel=[]; % N_cluster x 1
% cluster 
for idx=1:max(cluster_idx)
     this_cluster_idx=find(cluster_idx==idx);
        this_cluster_azim_occlusion=azim_occlusion(this_cluster_idx);
        this_cluster_elev_occlusion=elev_occlusion(this_cluster_idx);
        occlusion_cluster_centers=[ occlusion_cluster_centers ;[ mean(this_cluster_azim_occlusion) mean(this_cluster_elev_occlusion) ]];
        occlusion_cluster_numel=[occlusion_cluster_numel length(this_cluster_idx)];
end



global d_track nx ny
d_track=4;
nx=5; ny=4;


%% cost function check 
N_azim_sample=100; N_elev_sample=60;
azim_sample=linspace(0,2*pi,N_azim_sample);
elev_sample=linspace(elev_min,elev_max,N_elev_sample);

[azim_mesh,elev_mesh]=meshgrid(azim_sample,elev_sample);
azim_mesh=reshape(azim_mesh,[],1);
elev_mesh=reshape(elev_mesh,[],1);
cost_values=zeros(length(azim_mesh),1);
for i=1:length(azim_mesh)
    
    cost_values(i)=potential_field([azim_mesh(i) elev_mesh(i)],occlusion_cluster_centers,...
        occlusion_cluster_numel,3);
    if cost_values(i)>100
        cost_values(i)=100;
    end
end

figure()
surf(reshape(azim_mesh,N_elev_sample,N_azim_sample),...
    reshape(elev_mesh,N_elev_sample,N_azim_sample),...
    reshape(cost_values,N_elev_sample,N_azim_sample))

xlabel('azim'); ylabel('elev')
axis equal



%% problem 
p=[1 0 1]'; g=[3 1 0]';
w_d=5; % weight for tracking distance 
w_v=1; % weight for visibility 

figure
% tracker position
plot3(p(1),p(2),p(3),'ko')
hold on 
% target position 
plot3(g(1),g(2),g(3),'ro')

% bearing vector 
plot3([p(1) g(1)],[p(2) g(2)],[p(3) g(3)],'r-')

elev_cur=atan2(p(3)-g(3),norm(p(1:2)-g(1:2)));
azim_cur=atan2(p(2)-g(2),p(1)-g(1));

if azim_cur<0
   azim_cur=azim_cur+2*pi; 
end

x0=[norm(p-g) azim_cur elev_cur]';
view_vector=[x0(1)*cos(x0(3))*cos(x0(2)) x0(1)*cos(x0(3))*sin(x0(2)) x0(1)*sin(x0(3))]';
% bearing vector check : where I am ?
plot3([g(1)+view_vector(1)],[g(2)+view_vector(2)], [g(3)+view_vector(3)],'k*')


for i=1:length(x)
    elev=y(i); azim=x(i); r=z(i);
    view_vector=[r*cos(elev)*cos(azim) r*cos(elev)*sin(azim) r*sin(elev)]';
    if r>3.9
    %plot3([g(1) g(1)+view_vector(1)],[g(2) g(2)+view_vector(2)],[g(3) g(3)+view_vector(3)],'b','LineWidth',2)
    plot3([ g(1)+view_vector(1)],[ g(2)+view_vector(2)],[g(3)+view_vector(3)],'bo','LineWidth',2)
    else
    plot3([ g(1)+view_vector(1)],[ g(2)+view_vector(2)],[g(3)+view_vector(3)],'ro','LineWidth',2)
    end
    
end

axis equal

%% optimization 
options = optimoptions('fmincon','SpecifyObjectiveGradient',true,...
    'Display','iter','Algorithm','sqp');


% [f0,df0]=obj_fun(x0,p,g,w_d,w_v);
% pert=1e-6;
% pert_vec=pert*eye(3);
% df_r=obj_fun(x0+pert_vec(:,1),p,g,w_d,w_v)-f0;
% df_azim=obj_fun(x0+pert_vec(:,2),p,g,w_d,w_v)-f0;
% df_elev=obj_fun(x0+pert_vec(:,3),p,g,w_d,w_v)-f0;
% df=[df_r df_azim df_elev]'/pert;

lb=[0 0 elev_min+1e-3]; ub=[d_track+1 2*pi elev_max-1e-3];
X_sol=fmincon(@(X) obj_fun(X,p,g,w_d,w_v),x0,[],[],[],[],lb,ub,@(X) nonlcon_PWL(X),options);

%% plot 

r_sol=X_sol(1); azim=X_sol(2); elev=X_sol(3);
view_vector=[r_sol*cos(elev)*cos(azim) r_sol*cos(elev)*sin(azim) r_sol*sin(elev)]';
plot3([g(1) g(1)+view_vector(1)],[g(2) g(2)+view_vector(2)],[g(3) g(3)+view_vector(3)],'g-','LineWidth',3)
% does it satisfy the constrain ?
figure
[sf,~,output]=fit([x y],z,'linearinterp');
plot(sf,[x,y],z)
hold on 
plot3([mod(azim,2*pi) mod(azim,2*pi)],[elev elev],[0 r_sol],'r')









