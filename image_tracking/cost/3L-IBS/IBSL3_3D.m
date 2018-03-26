function P=IBSL3_3D(r,N,L,pnt,tri, normal)
%% ====================================================================
% Author: Mohammad Rouhani, Morpheo Team, INRIA Rhone Alpes, (2013)
% Email: mohammad.rouhani@inria.fr
% Title: convolutions between two B-Spline basis functions
% Place of publication: Grenoble, France
% Available from: URL
% http://www.iis.ee.ic.ac.uk/~rouhani/mycodes/IBS.rar
%====================================================================
% When using this software, PLEASE ACKNOWLEDGE the effort that went 
% into development BY REFERRING THE PAPER:
%::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: 
% Rouhani M. and Sappa A.D., Implicit B-spline fitting using the 3L 
% algorithm, IEEE Conference on on Image Processing (ICIP'11), 2011.
%::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: 
%% ====================================================================
%About this code:
%Input: pnt: points; tri: the triangular mesh; 
%       N: IBS size; r: the offset distance;
%Output: P: the IP coefficient vector

%This 3L algorithm is specially configurated for Implicit BSplines (IBS).
P=zeros(N,N,N);
%---------------------------------------------------------------
%addpath('D:\My Research\Surface Fitting\My Code\3D Surface\Algebraic\Higher degree')

x=pnt(:,1);y=pnt(:,2);z=pnt(:,3);

disp('The first step: computing the internal & external offset:')
%r=.4;%the offset distance
%%[s,t]=Offset3D2(r,x,y,z); hold on;
%load('BUNNY_OFF_005.mat');
%[s,t]=NewOffset(r,pnt, tri); hold on;
if (normal==0)
    [s,t]=NewOffset(r,pnt, tri); hold on;
else
    s=pnt+r*normal; t=pnt-r*normal;
end
if size(s,2)>3
    s=s'; t=t';
end
%[s,t]=NewOffset(r,pnt, tri); hold on; s=s'; t=t';
LetShow=0;
if LetShow
hold on
scatter3(s(:,1),s(:,2),s(:,3),'b','filled');
scatter3(pnt(:,1),pnt(:,2),pnt(:,3),'g','filled');
scatter3(t(:,1),t(:,2),t(:,3),'r','filled');
end
%--------------------------------
%definition of variables:
n=numel(x);
%M=size(P,1); N=size(P,2); O=size(P,3);
b=-0*ones(n,1);%warning: it must be -1
%MM=zeros(n,N^3);
%define the offset distance and the expected value
eps=1.0; 
b=[b; b+eps; b-eps];
tic
%---------------------------------------------------------------
disp('The second step: Computing the control values of IBS.')
w1=waitbar(0,'The monomial matrix is being constructed');
tic;
waitbar(1/3,w1); M0=BlockMatrix3D(P,x,y,z);                M0=sparse(M0);
waitbar(2/3,w1); MP=BlockMatrix3D(P,s(:,1),s(:,2),s(:,3)); MP=sparse(MP);
waitbar(3/3,w1); MN=BlockMatrix3D(P,t(:,1),t(:,2),t(:,3)); MN=sparse(MN);
close(w1); Time2=toc; 
MM=[M0; MP; MN];
clear M0; clear MP; clear MN;

%title(cond(MM));
%c=inv(MM'*MM)*(MM'*b);
%vP=inv(MM'*MM+L*eye(N^3))*(MM'*b); 
MatName=['H',num2str(N),'.mat'];
if exist(MatName)
    load(MatName);
else
    H=RegMatrix3D(N);
end
H=sparse(H);
%H=eye(N^3);
H=sparse(H); MM=sparse(MM);
MAT=sparse(MM'*MM+ L*H); bb=(MM'*b);
tic;
vP=MAT\bb; %vP=inv(MM'*MM+ L*H)*(MM'*b); 
Time_LS=toc 
% This is in the vector form; it should be converted to the matrix form.
%Matlab standard order: layer-by-layer, column-by...
P(:)=vP; %P=reshape(vP,N,N,N);
DIM=N^3;
disp(['%% IBS Dimension: ',num2str(DIM),' & CPU time for LS: ',num2str(Time_LS)]); 
%--------------------------------
disp('Please call IBSLevelSurf function for rendering the result.')
close all
IBSLevelSurf(P,pnt,200);
% for i=1:M
%     for j=1:N
%         scatter3(i/M,j/N,P(i,j),'filled'); %it is NOT precisely located. 
%     end
% end
%----------------------------------puting additional constraint:
%just one: on p_31
% i=9;j=10;k=11;  W=[Dfc(x(i),y(i)); Dfc(x(j),y(j)); Dfc(x(k),y(k)) ];
% lambda=-2*inv(W*inv(M'*M)*W')*W*inv(M'*M)*M'*b;
% c=inv(M'*M)*(M'*b)+.5*inv(M'*M)*W'*lambda;
numIND=N^3;
%polyplot(c,-20,20)

function MM=BlockMatrix3D(P,x,y,z)
%%
n=numel(x); N=size(P,1); %N=size(P,2); O=size(P,3); 
stepX=1/(N-3);
MM=zeros(n,N^3);

M=size(P,1); N=size(P,2); O=size(P,3);
stepX=1/(M-3); stepY=1/(N-3); stepZ=1/(O-3);

%Every point [x,y,z] is projected to [u,v,w]-space:
i=floor(x/stepX)+1; j=floor(y/stepY)+1; k=floor(z/stepZ)+1;
u=x/stepX-i+1; v=y/stepY-j+1; w=z/stepZ-k+1;
%Blending patches together
B=[-1 3 -3 1; 3 -6 3 0; -3 0 3 0; 1 4 1 0]/6;
bu=[u.^3 u.^2 u 1+0*u]*B; bv=[v.^3 v.^2 v 1+0*v]*B; bw=[w.^3 w.^2 w 1+0*w]*B;
for ii=0:3
    for jj=0:3
        for kk=0:3 
            %Col=N^2*(i+ii-1)+N*(j+jj-1)+(k+kk);
            %Matlab standard order: layer-by-layer, column-by...
            Col=N^2*(k+kk-1)+N*(j+jj-1)+(i+ii);
            %MM(:,Col)=bu(:,ii).*bv(:,jj).*bw(:,kk); %it doesn't work
            ind=(Col-1)*n+(1:n)';
            MM(ind)=bu(:,ii+1).*bv(:,jj+1).*bw(:,kk+1);
        end
    end
end