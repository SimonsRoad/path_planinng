function res=obj_fun_SEDT(x)
% this object function includes SEDT as Qv
% D = SEDT field 
global p g w_d w_v coeff_surf d_track
if length(x)>2
r=x(1); azim=x(2); elev=x(3);
else
r=d_track; azim=x(1); elev=x(2);    
end
translational_cost=norm(g-p+[r*cos(elev)*cos(azim) r*cos(elev)*sin(azim) r*sin(elev)]')^2;
tracking_distance_cost=w_d*(r-d_track)^2;
visibility_cost=w_v *poly_surf_val([azim elev] ,coeff_surf,5,3);
fprintf("------------------------------- \n")
fprintf("current r: %d azim %d elev %d\n",r,azim,elev);
fprintf( "transitional cost : %d \n",translational_cost);
fprintf( "tracking distance cost : %d\n",tracking_distance_cost);
fprintf( "visibility cost : %d\n",visibility_cost);
fprintf("------------------------------- \n")

res=translational_cost+tracking_distance_cost+visibility_cost;
end


