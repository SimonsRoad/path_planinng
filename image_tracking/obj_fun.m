function [res,grad]=obj_fun(X,p,g,w_d,w_v)
% this function computes the objective function value of given params
% X=[r azim elev] (3 x 1)
% p: position of tracker (3 x 1)
% g: position of target  (3 x 1)
% d_track : desired tracking distance 
% w_d : weight for tracking distance 
% w_v : weight for visibility distance 

global d_track castRayResult param 
r=X(1); azim=X(2); elev=X(3);

% it is modular operation 
azim=mod(azim,2*pi);

% parsing parameters 
azim_min=param(1);
azim_max=param(2);
N_azim=param(3);  D_azim=(azim_max-azim_min)/(N_azim-1);
azim_set=linspace(azim_min,azim_max,N_azim);

elev_min=param(4); 
elev_max=param(5); 
N_elev=param(6); D_elev=(elev_max-elev_min)/(N_elev-1);
elev_set=linspace(elev_min,elev_max,N_elev);

[x,y]=meshgrid(azim_set,elev_set);
z=castRayResult;

x=reshape(x,[],1);
y=reshape(y,[],1);
z=reshape(z,[],1);

%% translational cost / tracking distance cost  
translational_cost=norm(g-p+[r*cos(elev)*cos(azim) r*cos(elev)*sin(azim) r*sin(elev)]')^2;
tracking_distance_cost=w_d*(r-d_track)^2;
grad=zeros(3,1);

grad(1)=2*[cos(elev)*cos(azim) cos(elev)*sin(azim) sin(elev)]*...
    (g-p+[r*cos(elev)*cos(azim) r*cos(elev)*sin(azim) r*sin(elev)]')+w_d*2*(r-d_track); 

grad(2)=2*[-r*cos(elev)*sin(azim) r*cos(elev)*cos(azim) 0]*(g-p+[r*cos(elev)*cos(azim) r*cos(elev)*sin(azim) r*sin(elev)]');

grad(3)=2*[-r*sin(elev)*cos(azim) -r*sin(elev)*sin(azim) r*cos(elev)]*(g-p+[r*cos(elev)*cos(azim) r*cos(elev)*sin(azim) r*sin(elev)]');


query_azim_elev_pair=[azim elev]';


%% visibility cost 
visibility_cost=0;
decaying_factor=0.2; % bigger : smoother 
norm_length_azim=2*pi;
norm_length_elev=elev_max-elev_min;
query_azim_elev_pair_normalized=[(azim-azim_min)/norm_length_azim ...
        (elev-elev_min)/norm_length_elev]';
    
for i =1: length(x)

    % sampled rays 
    cur_azim=x(i); cur_elev=y(i);
    cur_azim_norm=cur_azim/norm_length_azim;
    cur_elev_norm=(cur_elev-elev_min)/norm_length_elev;
    
    dist=norm(query_azim_elev_pair_normalized-[cur_azim_norm cur_elev_norm]');
    cur_visibility_cost=w_v*exp(-dist/decaying_factor)*(d_track-z(i))^2;
    visibility_cost=visibility_cost+cur_visibility_cost;    
    grad(2)=grad(2)+cur_visibility_cost...
        /(-decaying_factor)*(query_azim_elev_pair_normalized(1)-cur_azim_norm)/dist/norm_length_azim;    
    grad(3)=grad(3)+cur_visibility_cost...
        /(-decaying_factor)*(query_azim_elev_pair_normalized(2)-cur_elev_norm)/dist/norm_length_elev;

end

res=translational_cost+tracking_distance_cost+visibility_cost;

fprintf("------------------------------- \n")
fprintf("current r: %d azim %d elev %d\n",r,azim,elev);
fprintf( "transitional cost : %d \n",translational_cost);
fprintf( "tracking distance cost : %d\n",tracking_distance_cost);
fprintf( "visibility cost : %d\n",visibility_cost);

fprintf("------------------------------- \n")


end
