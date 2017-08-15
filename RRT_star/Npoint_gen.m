function pnts=Npoint_gen(q,l,N)
% N point generation of drone flatform 
% q = xyz_rpy 
% 4*N+2 = # of points
% pnts= 3 x (4N+2)
rot_mat=rotx(q(4))*roty(q(5))*rotz(q(6)); t=[q(1) q(2) q(3)]'; 
T=SE3(rot_mat,t);
R=double(T);
pnts=[pnt_gen([-l 0 0 1],[l 0 0 1],2*N+1) pnt_gen([0 -l 0 1],[0 l 0 1],2*N+1)];
pnts=R*pnts;
pnts=pnts(1:3,:);
end


