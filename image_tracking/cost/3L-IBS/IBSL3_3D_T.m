function P=IBSL3_3D_T(r,N,L,pnt,tri, normal)
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
TPS_Regularizer=0;
P=zeros(N,N,N);
ShowOffset=0;
%---------------------------------------------------------------
addpath('D:\My Research\Surface Fitting\My Code\3D Surface\Algebraic\Higher degree');

x=pnt(:,1);y=pnt(:,2);z=pnt(:,3);

disp('-----1ST STEP: computing the offsets.-----')
%r=.4;%the offset distance
%%[s,t]=Offset3D2(r,x,y,z); hold on;
%load('my_face3_005.mat'); s=S; t=T;%load('MYFACE_OFF_005.mat');
if tri==0
    s=pnt+r*normal; ss=pnt+2*r*normal; t=pnt-r*normal;
else
    [s,t,normal]=NewOffset(r,pnt, tri); ss=pnt+2*r*normal;
end
%[s,t]=NewOffset2(r,pnt, tri); hold on;
if size(s,2)>3
    s=s'; t=t';
end
%[s,t]=NewOffset(r,pnt, tri); hold on; s=s'; t=t';
if ShowOffset
    hold on
    scatter3(s(:,1),s(:,2),s(:,3),'b.');
    scatter3(pnt(:,1),pnt(:,2),pnt(:,3),'g.');
    scatter3(t(:,1),t(:,2),t(:,3),'r.');
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
disp('-----2ND STEP: Construct the IBS basis matrix.-----')
SET=[pnt; s; t];
IND=activeINDEX(P,SET);
DIM=max(IND(:)); %The dimension of truncated IBS
tic;
M0=BlockMatrix3D_T(IND,P,x,y,z);                M0=sparse(M0);
MP=BlockMatrix3D_T(IND,P,s(:,1),s(:,2),s(:,3)); MP=sparse(MP);
MN=BlockMatrix3D_T(IND,P,t(:,1),t(:,2),t(:,3)); MN=sparse(MN);
MM=[M0; MP; MN];
toc %Time_MAT
clear M0; clear MP; clear MN;
%title(cond(MM));

tic;
if TPS_Regularizer
    disp('Regularization term: TPS');    
    H=RegMatrix3D_T(N,IND); L=10^2; %H=RegMatrix3D(N);
else
    disp('Regularization term: Tikhonov (Identity)');
    H=sparse(DIM); for k=1:DIM; H(k,k)=1; end    %H=eye(DIM); H=sparse(H);
end
toc %Time_REG

disp('-----3RD STEP: Solving the Least Squares.-----')
MAT=MM'*MM+ L*H; MAT=sparse(MAT);
tic;
vP=MAT\(MM'*b); %vP=inv(MM'*MM+ L*H)*(MM'*b); 
Time_LS=toc;
disp(['%% IBS-T Dimension: ',num2str(DIM),' & CPU time for LS: ',num2str(Time_LS)]); 
% This is in the vector form; it should be converted to the matrix form.
%P=InsidOutside(N,pnt,normal); %Inside -10, Outside +10
P=zeros(N,N,N)+10;
v=find(IND>0);
P(v)=vP;

%--------------------------------
disp('END: Representing by IBSLevelSurf...')
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
%polyplot(c,-20,20)


function IND=activeINDEX(P,SET)
%%
x=SET(:,1); y=SET(:,2); z=SET(:,3);
n=size(SET,1); M=size(P,1); N=size(P,2); O=size(P,3); 
IND=zeros(N,N,N);
stepX=1/(M-3); stepY=1/(N-3); stepZ=1/(O-3);

%Every point [x,y,z] is projected to [u,v,w]-space:
i=floor(x/stepX)+1; j=floor(y/stepY)+1; k=floor(z/stepZ)+1;
for h=1:numel(i)    
    IND(i(h)+(0:3),j(h)+(0:3),k(h)+(0:3))=1;
end
v=find(IND(:)>0);
IND(v)=1:numel(v);

function MM=BlockMatrix3D_T(IND,P,x,y,z)
%%
DIM=max(IND(:)); n=numel(x);
M=size(P,1); N=size(P,2); O=size(P,3);
stepX=1/(M-3); stepY=1/(N-3); stepZ=1/(O-3);

%MM=zeros(n,N^3); IT TAKES LOTS OF MEMORY
MM=zeros(n,DIM);
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
            %THIS IS THE DIFFERENCE WITH NORMAL IBS:::::::::::
            Col=IND(Col); %using the truncated indexing! :)
            
            %MM(:,Col)=bu(:,ii).*bv(:,jj).*bw(:,kk); %it doesn't work
            ind=(Col-1)*n+(1:n)';
            MM(ind)=bu(:,ii+1).*bv(:,jj+1).*bw(:,kk+1);
        end
    end
end

function P=InsidOutside(N,pnt,normal); %Inside -10, Outside +10
pnt0=pnt; normal0=normal;
sampling=10; pnt=pnt(1:sampling:end,:); normal=normal(1:sampling:end,:);
P=zeros(N,N,N)+10;
stepX=1/(N-3);
cnt=stepX*(-1:N-2); [Y,X,Z]=meshgrid(cnt,cnt,cnt);
CNT(:,1)=X(:); CNT(:,2)=Y(:); CNT(:,3)=Z(:);  

D=pdist2(pnt,CNT);
[mn closest]=min(D);
vec1=CNT-pnt(closest,:); vec2=normal(closest,:);
%simply find if the angle of these two vector is more than 90 degree:
innerProd=sum(vec1'.*vec2')'./sum(vec1'.*vec1')'.^0.5;
v=find(innerProd<-0.5); P(v)=-10; %These points are INSIDE!!
 
function showLattic(P,IND)
%%
N=size(P,1);
stepX=1/(N-3);
X=-stepX:stepX:1+stepX; Y=X; Z=X;
color1=[1 2 1]*.5; color2=[1 2 1]*.5;
for i=1:N-1
    for j=1:N-1
        for k=1:N-1            
            if IND(i,j,k)*IND(i+1,j,k)
                plot3([X(i) X(i+1)],[Y(j) Y(j)], [Z(k) Z(k)],'Color',color2,'LineWidth',1);
            end
            
            if IND(i,j,k)*IND(i,j+1,k)
                plot3([X(i) X(i)],[Y(j) Y(j+1)],[Z(k) Z(k)],'Color',color2,'LineWidth',1);
            end
            if IND(i,j,k)*IND(i,j,k+1)
                plot3([X(i) X(i)],[Y(j) Y(j)],[Z(k) Z(k+1)],'Color',color2,'LineWidth',1);
            end
        end
    end
end