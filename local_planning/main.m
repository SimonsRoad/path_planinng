% minimum snap trajectory generation 
% parameters: 
% m= # of key frame(set point) => ts=[t0 t1 t2 ... tm]
% n= order of polynomials
% parameter =
% [p1 p2 p3 p4 ... pn] pi=[pi1 pi2 pi3 pi4 ... pim] 
% pij=[rij kji]  
n=6; ts=[0 1 2 3 4];
m=length(ts)-1;
p=ones(4*(n+1)*m);
C=zeros(n,n,m);
M=zeros(4*(n+1)*m);
for k=1:m
    for i=4:n
        for j=4:n
            if i==4 && j==4
            C(i,j,k)=i*(i-1)*(i-2)*(i-3)*j*(j-1)*(j-2)*(j-3)*(ts(k+1)-ts(k));
            else
            C(i,j,k)=i*(i-1)*(i-2)*(i-3)*j*(j-1)*(j-2)*(j-3)*1/(i+j-8)*(ts(k+1)^(i+j-7)-ts(k)^(i+j-7));
            end
        end
    end
end

for i=1:n
    for j=1:n
        M(4*i-3:4*i,4*j-3:4*j)=
    
    end
end

