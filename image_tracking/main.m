%% data generation 
global castRayResult param

load example_raycast_result.mat mat
castRayResult=mat;
elev_min=pi/8; elev_max=pi/3;
param=[0 2*pi size(castRayResult,2) elev_min elev_max size(castRayResult,1) ];
[x,y]=meshgrid(linspace(0,2*pi,size(castRayResult,2)),linspace(elev_min,elev_max,size(castRayResult,1)));
z=castRayResult;
x=reshape(x,[],1);
y=reshape(y,[],1);
z=reshape(z,[],1);

% x=[x ; x+2*pi];
% y=[y;y];
% z=[z;z];
% 
% x=[x ; x-2*pi];
% y=[y;y];
% z=[z;z];
% 
% x=x/(2*pi);
% y=(y-elev_min)/(elev_max-elev_min);

global d_track nx ny
d_track=4;
nx=5; ny=4;



%% visibility cost formulation


query_point=[0.5,0.8];
x_train=[];
y_train=[];
z_train=[];
for i=1:length(x)
   
    if norm([x(i) y(i)]-[query_point(1) query_point(2)])<0.5
        x_train=[x_train; x(i)];
        y_train=[y_train ;y(i)];
        z_train=[z_train ;z(i)];
        
    end
    
    
end

% sf=fit([x_train y_train],z_train/4,'poly42');

% [sf,~,output]=fit([x y],z,'linearinterp');
% coeff=coeffvalues(sf);
plot(sf,[x,y],z)
xlabel('azim'); ylabel('elev')
hold on 

plot3(query_point(1),query_point(2),4,'r*');
%plot3(query_azim_elev_pair(1),query_azim_elev_pair(2),4.1,'ro','LineWidth',3) % query point ? 
costs=[];
N_azim_sample=100; N_elev_sample=60;
azim_sample=linspace(0,2*pi,N_azim_sample);
elev_sample=linspace(elev_min,elev_max,N_elev_sample);

[azim_mesh,elev_mesh]=meshgrid(azim_sample,elev_sample);
azim_mesh=reshape(azim_mesh,[],1)+0.01*rand(N_azim_sample*N_elev_sample,1);
elev_mesh=reshape(elev_mesh,[],1)+0.01*rand(N_azim_sample*N_elev_sample,1);
%% cost3: quadratic based cost function 

decaying_factor=0.1;
translational_set=[-2*pi 0 2*pi];

for q_idx=1:length(azim_mesh)
   query_azim_elev_pair_normalized=[azim_mesh(q_idx)/(2*pi) (elev_mesh(q_idx)-elev_min)/(elev_max-elev_min)]';
    visibility_cost=0;
    for translate=translational_set
    for i=1:length(x)
        cur_azim=x(i)+translate; cur_elev=y(i);
        sampled_azim_elev_pair_normalized=[cur_azim/(2*pi) (cur_elev-elev_min)/(elev_max-elev_min)]';
        dist=norm(query_azim_elev_pair_normalized-[cur_azim/(2*pi) (cur_elev-elev_min)/(elev_max-elev_min)]');

        visibility_cost=-dist^2;
    end
    end
    costs=[costs visibility_cost]; 
end

%% result drawing
figure()
surf(reshape(azim_mesh,N_elev_sample,N_azim_sample),...
    reshape(elev_mesh,N_elev_sample,N_azim_sample),...
    reshape(costs,N_elev_sample,N_azim_sample))
xlabel('azim'); ylabel('elev')

%% cost2: exponential based cost function 

costs=[];
N_azim_sample=100; N_elev_sample=60;
azim_sample=linspace(0,2*pi,N_azim_sample);
elev_sample=linspace(elev_min,elev_max,N_elev_sample);

[azim_mesh,elev_mesh]=meshgrid(azim_sample,elev_sample);
azim_mesh=reshape(azim_mesh,[],1)+0.01*rand(N_azim_sample*N_elev_sample,1);
elev_mesh=reshape(elev_mesh,[],1)+0.01*rand(N_azim_sample*N_elev_sample,1);


decaying_factor=0.1;
translational_set=[-2*pi 0 2*pi];

for q_idx=1:length(azim_mesh)
   query_azim_elev_pair_normalized=[azim_mesh(q_idx)/(2*pi) (elev_mesh(q_idx)-elev_min)/(elev_max-elev_min)]';
    visibility_cost=0;
    for translate=translational_set
    for i=1:length(x)
        cur_azim=x(i)+translate; cur_elev=y(i); cur_r=z(i);
        sampled_azim_elev_pair_normalized=[cur_azim/(2*pi) (cur_elev-elev_min)/(elev_max-elev_min)]';
        dist=norm(query_azim_elev_pair_normalized-[cur_azim/(2*pi) (cur_elev-elev_min)/(elev_max-elev_min)]');
        if cur_r <1
        visibility_cost=visibility_cost+exp(-dist/decaying_factor)*(d_track-z(i))^2;
        end
    end
    end
    costs=[costs visibility_cost];
 
end

%% result drawing
figure()
surf(reshape(azim_mesh,N_elev_sample,N_azim_sample),...
    reshape(elev_mesh,N_elev_sample,N_azim_sample),...
    reshape(costs,N_elev_sample,N_azim_sample))
xlabel('azim'); ylabel('elev')


%% cost1 : rational function 
for q_idx=1:length(azim_mesh)

query_azim_elev_pair=[azim_mesh(q_idx) elev_mesh(q_idx)]';
% impact zone 
kernel_x=2*pi/3; kernel_y=(elev_max-elev_min)/3;
potential_idces=[];
potential_idces_prev_period=[]; % the previous -2*pi 
potential_idces_next_period=[]; % the next 2*pi 
visibility_cost=0;
count=0;
for i=1:length(x)
    cur_azim=x(i); cur_elev=y(i);
    % cur sampled point is inside of kernel 
    
    if query_azim_elev_pair(1)-kernel_x/2<cur_azim && query_azim_elev_pair(1)+kernel_x/2>cur_azim ...
            && query_azim_elev_pair(2)-kernel_y/2<cur_elev && query_azim_elev_pair(2)+kernel_y/2>cur_elev 
        this_cost=(norm(query_azim_elev_pair-[cur_azim cur_elev]'))^-2*(d_track- z(i));
        
        if this_cost>100
           this_cost=100 ;
        end
        visibility_cost=visibility_cost+this_cost;
    end
    
    cur_azim=x(i)-2*pi; 
    if query_azim_elev_pair(1)-kernel_x/2<cur_azim && query_azim_elev_pair(1)+kernel_x/2>cur_azim ...
           && query_azim_elev_pair(2)-kernel_y/2<cur_elev && query_azim_elev_pair(2)+kernel_y/2>cur_elev
        
       this_cost=(norm(query_azim_elev_pair-[cur_azim cur_elev]'))^-2*(d_track- z(i));

        if this_cost>100
           this_cost=100 ;
        end
       
       
        visibility_cost=visibility_cost+this_cost;

    end
    
    cur_azim=x(i)+2*pi; 
    if query_azim_elev_pair(1)-kernel_x/2<cur_azim && query_azim_elev_pair(1)+kernel_x/2>cur_azim ...
           && query_azim_elev_pair(2)-kernel_y/2<cur_elev && query_azim_elev_pair(2)+kernel_y/2>cur_elev
        this_cost=(norm(query_azim_elev_pair-[cur_azim cur_elev]'))^-2*(d_track- z(i));

        if this_cost>100
           this_cost=100 ;
        end
       
        
        visibility_cost=visibility_cost+this_cost;   
        
    end
    
end

costs=[costs visibility_cost];

end
%% result drawing
figure()
surf(reshape(azim_mesh,N_elev_sample,N_azim_sample),...
    reshape(elev_mesh,N_elev_sample,N_azim_sample),...
    reshape(costs,N_elev_sample,N_azim_sample))
xlabel('azim'); ylabel('elev')


%% problem 
p=[3 0 1]'; g=[3 1 0]';
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

lb=[0 -inf elev_min+1e-3]; ub=[d_track+1 inf elev_max-1e-3];
X_sol=fmincon(@(X) obj_fun(X,p,g,w_d,w_v),x0,[],[],[],[],lb,ub,@(X) nonlcon_PWL(X),options);

%% plot 

r_sol=X_sol(1); azim=X_sol(2); elev=X_sol(3);
view_vector=[r_sol*cos(elev)*cos(azim) r_sol*cos(elev)*sin(azim) r_sol*sin(elev)]';
plot3([g(1) g(1)+view_vector(1)],[g(2) g(2)+view_vector(2)],[g(3) g(3)+view_vector(3)],'g-','LineWidth',3)
% does it satisfy the constrain ?
figure
plot(sf,[x,y],z)
hold on 
plot3([mod(azim,2*pi) mod(azim,2*pi)],[elev elev],[0 r_sol],'r')









