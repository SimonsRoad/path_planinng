function xdot=CoopQuad2Dynmaics(t,x,data)
% See 연구노트2 권 p4 
% state description x=[xl vl Rl wl] u=[qi_(i=1..4) Ti_(i=1..4)]
% data : ml / Jl

% Data extraction 
ml=data.ml; 
Jl=data.Jl;  % 3x3 symmetric matrix
ri=data.ri;   %3x4 ri matrix 
% State extraction
xl=x(1:3); %2
vl=x(4:6); %2
Rl=reshape(x(7:15),3,3);  %4
wl=x(16:18);    %1 
u=data.u;

Q=reshape(u(1:12),3,4);
for i=1:4
   Q(:,i) =Q(:,i)/norm(Q(:,i));
end
T=u(13:16);

xldot=vl;
vldot=Rl*sum(Q.*repmat(T',3,1),2)/ml;
wldot=Jl\(-cross(wl,Jl*wl)+sum(cross(ri,Q.*repmat(T',3,1)),2));
Rldot=Rl*skew(wl);

xdot = [xldot;
      vldot;
      reshape(Rldot, 9,1) ;
      wldot];

end