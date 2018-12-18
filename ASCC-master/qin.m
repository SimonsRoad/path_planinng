function out=qin(qs)
% q= theta: nbData x dof
% q2  nbData x 2dof  

[nbData,dof]=size(qs);
out=zeros(nbData,2*dof);
for i=1:dof
    out(:,2*i-1)=qs(:,i);
end

end