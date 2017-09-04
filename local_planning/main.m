% minimum snap trajectory generation 
% 연구노트 66p 


% parameters: 
% m= # of key frame(set point) => ts=[t0 t1 t2 ... tm]
% n= order of polynomials
% parameter =
% [p1 p2 p3 p4 ... pn] pi=[pi1 pi2 pi3 pi4 ... pim] 
% pij=[rij kji]  


%% objective function 
global m n ts
n=6; ts=[0 1 2 3 4];
m=length(ts)-1;
C=zeros(n,n,m);


M=zeros(4*(n+1)*m);
keyframe=zeros(4,5); % [r_T yaw_T]'*(# of keyframe)

for i=1:5
    keyframe(:,i)=(i-1)*ones(4,1);
end

% position objective

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
        Mij=zeros(4*m);
        for k=1:m
            Mij(4*k-3:4*k,4*k-3:4*k)=C(i,j,k)*blkdiag(eye(3),0);
        end
        M(4*m*(i)+1:4*m*(i+1),4*m*(j)+1:4*m*(j+1))=Mij;    
    end
end

Mr=M;
%%
% yaw objective
C=zeros(n,n,m);

for k=1:m
    for i=2:n
        for j=2:n
            if i==2 && j==2
            C(i,j,k)=i*(i-1)*j*(j-1)*(ts(k+1)-ts(k));
            else
            C(i,j,k)=i*(i-1)*j*(j-1)*1/(i+j-4)*(ts(k+1)^(i+j-3)-ts(k)^(i+j-3));
            end
        end
    end
end


for i=1:n
    for j=1:n
<<<<<<< HEAD
        Mij=zeros(4*m);
        for k=1:m
            Mij(4*k-3:4*k,4*k-3:4*k)=C(i,j,k)*blkdiag(0,eye(3));
        end
        M(4*m*(i)+1:4*m*(i+1),4*m*(j)+1:4*m*(j+1))=Mij;    
=======
        M(4*i-3:4*i,4*j-3:4*j)=
        
>>>>>>> 938cea06db4b62c0507bc70799015cda2ae17d62
    end
end

M_ksi=M;




%% constraint matrix for continous 
T=[];

for i=1:n+1 % actually from 0 to n
    Ti=zeros(4*m);
    for k=1:m % actually from 0 to m
        Ti(4*(k-1)+1:4*(k),4*(k-1)+1:4*(k))=ts(k)^(i-1)*eye(4);
    end
    k=m+1;
        Ti(4*(k-1)+1:4*(k),4*(k-2)+1:4*(k-1))=ts(k)^(i-1)*eye(4);
    T=[T Ti];
end

%% optimization 
keyframe=reshape(keyframe,numel(keyframe),1);
p=quadprog(Mr+M_ksi,[],[],[],T,keyframe);







%% plot the trajectory 





figure
plot3(keyframe(1,:),keyframe(2,:),keyframe(3,:),'r*')
hold on 
axis([0 10 0 10 0 10])


