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
%% fitting 
% sf=fit([x y],z,'poly54');
[sf,~,output]=fit([x y],z,'linearinterp');
% coeff=coeffvalues(sf);
plot(sf,[x,y],z)
% xlabel('azim'); ylabel('elev')


%% problem 
p=[3 0 1]'; g=[3 1 0]'; w=10; % weight for visibiltiy  
figure
plot3(p(1),p(2),p(3),'ko')
hold on 
plot3(g(1),g(2),g(3),'ro')
plot3([p(1) g(1)],[p(2) g(2)],[p(3) g(3)],'r-')
for i=1:length(x)
    elev=y(i); azim=x(i); r=z(i);
    view_vector=[r*cos(elev)*cos(azim) r*cos(elev)*sin(azim) r*sin(elev)]';
    if r<3.9
    %plot3([g(1) g(1)+view_vector(1)],[g(2) g(2)+view_vector(2)],[g(3) g(3)+view_vector(3)],'b','LineWidth',2)
    plot3([ g(1)+view_vector(1)],[ g(2)+view_vector(2)],[g(3)+view_vector(3)],'bo','LineWidth',2)
    end
end

%% optimization 
options = optimoptions('fmincon','SpecifyObjectiveGradient',true,...
    'Display','off','Algorithm','sqp');

global d_track nx ny
d_track=4;
nx=5; ny=4;

elev_cur=atan2(p(3)-g(3),norm(p(1:2)-g(1:2)));
azim_cur=atan2(p(2)-g(2),p(1)-g(1));

if azim_cur<0
   azim_cur=azim_cur+2*pi; 
end

lb=[0 0 elev_min]; ub=[d_track+1 2*pi elev_max];
x0=[norm(p-g) azim_cur elev_cur]';
X_sol=fmincon(@(X) obj_fun(X,p,g,w),x0,[],[],[],[],lb,ub,@(X) nonlcon_PWL(X),options);

%% plot 

r_sol=X_sol(1); azim=X_sol(2); elev=X_sol(3);
view_vector=[r_sol*cos(elev)*cos(azim) r_sol*cos(elev)*sin(azim) r_sol*sin(elev)]';
plot3([g(1) g(1)+view_vector(1)],[g(2) g(2)+view_vector(2)],[g(3) g(3)+view_vector(3)],'r')
% does it satisfy the constrain ?
figure
plot(sf,[x,y],z)
hold on 
plot3([azim azim],[elev elev],[0 r_sol],'r')









