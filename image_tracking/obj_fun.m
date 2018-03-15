function [res,grad]=obj_fun(X,p,g,w)
% this function computes the objective function value of given params
% X=[r azim elev] (3 x 1)
% p: position of tracker (3 x 1)
% g: position of target  (3 x 1)
% d_track : desired tracking distance 
global d_track castRayResult param 
r=X(1); azim=X(2); elev=X(3);


% parsing parameters 
azim_min=param(1);
azim_max=param(2);
N_azim=param(3);  D_azim=(azim_max-azim_min)/(N_azim-1);
azim_set=linspace(azim_min,azim_max,N_azim);

elev_min=param(4); 
elev_max=param(5); 
N_elev=param(6); D_elev=(elev_max-elev_min)/(N_elev-1);
elev_set=linspace(elev_min,elev_max,N_elev);

azim_elev_pair=meshgrid(azim_set,elev_set);




res=norm(g-p+[r*cos(elev)*cos(azim) r*cos(elev)*sin(azim) r*sin(elev)]')^2+w*(r-d_track)^4;

fprintf("transitional cost : %d / tracking distance cost : %d\n",res-w*(r-d_track)^2,w*(r-d_track)^2 );

grad=zeros(3,1);

grad(1)=2*[cos(elev)*cos(azim) cos(elev)*sin(azim) sin(elev)]*...
    (g-p+[r*cos(elev)*cos(azim) r*cos(elev)*sin(azim) r*sin(elev)]')+w*2*(r-d_track); 

grad(2)=2*[-r*cos(elev)*sin(azim) r*cos(elev)*cos(azim) 0]*(g-p+[r*cos(elev)*cos(azim) r*cos(elev)*sin(azim) r*sin(elev)]');

grad(3)=2*[-r*sin(elev)*cos(azim) -r*sin(elev)*sin(azim) r*cos(elev)]*(g-p+[r*cos(elev)*cos(azim) r*cos(elev)*sin(azim) r*sin(elev)]');



end
