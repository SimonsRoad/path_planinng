function [x0,J0,x0dot,do]=MK2(q)
% Maciejewski and Klein proposed
% q is dof x 1 or 1 x dof joint angle

global arms arms_flat obs
global dof

% finding current pos of candidate points 
q2=qin(q);


[T_end T_all]=arms{dof}.end.fkine(q2);
xs=zeros(3,length(T_all));
nbCand=length(T_all);

for i=1:nbCand
xs(:,i)=T_all(i).t;
end

% calculating relative distance
ds_tmp=repmat(obs',1,nbCand)-xs;
for i=1:nbCand
ds(i)=norm(ds_tmp(:,i),2);
end

% truncate points attached to end effector link
% ds=ds(1:end-3);
[do,nrst]=min(ds);

disp(nrst)
D=3;

% finding Jacobian 
J0=arms_flat{nrst}.jacob0(q2(1:nrst));
J0=J0(1:D,1:2:nrst); 
J0=[J0  zeros(D,dof-size(J0,2))];


% potential field 
x0=xs(:,nrst);
x0dot=Calc_Psi(x0);




end