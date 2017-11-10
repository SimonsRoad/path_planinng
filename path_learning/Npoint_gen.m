function pnts=Npoint_gen(q,r,N)
% N point generation of in the form of disc 
% q = xyz_rpy 
% see p2 in 연구노트2
% number of generated points is N+1 

pnts=zeros(3,N+1);
pnts(:,1)=zeros(3,1);
rot_mat=rotx(q(4))*roty(q(5))*rotz(q(6)); t=[q(1) q(2) q(3)]'; 
T=SE3(rot_mat,t);
delta=2*pi/N;
for i=1:N
    pnts(:,i+1)=[r*cos(delta*(i-1)) ; r*sin(delta*(i-1)) ; 0 ];
end
pnts=T*pnts;
    
 
end


