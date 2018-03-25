function IBSLevelSurf(P,pnt,RES)
%% ====================================================================
% Author: Mohammad Rouhani, Morpheo Team, INRIA Rhone Alpes, (2013)
% Email: mohammad.rouhani@inria.fr
% Title: IBS value calculation.
% Place of publication: Grenoble, France
% Available from: URL
% http://www.iis.ee.ic.ac.uk/~rouhani/mycodes/Geometric.zip
%====================================================================
% When using this software, PLEASE ACKNOWLEDGE the effort that went 
% into development BY REFERRING THE PAPER:
%::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: 
% Rouhani M. and Sappa A.D., Implicit B-spline ?tting using the 3L 
% algorithm, IEEE Conference on on Image Processing (ICIP'11), 2011.
%::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: 
%% ====================================================================
%About this code:
%Input: P: IBS coefficients; [x,y,z]: point coordinates.
%Output: f: IBS value at the given points; Buvw: IBS monomials.
ShowOtherLevels=0;
run_mine=0;
%surface plot:---------
%Xrange=-0:.1:10; Yrange=-0:.1:10;
box=[.1 .9 .1 .9 .1 .9];
box(1:2:5)=min(pnt)-.05; box(2:2:6)=max(pnt)+.05;
%box=[ .1 .85 .4 .6 .15 .85]
%Xrange=0:step:boundx; Yrange=0:step:boundy; Zrange=0:step:boundz;
step=1/RES;
Xrange=box(1):step:box(2); Yrange=box(3):step:box(4); Zrange=box(5):step:box(6);

%Xrange=-0:step:boundx; Yrange=-0-.2:step:boundy; Zrange=-0:step:boundz;
%Xrange=-.8:step:.8; Yrange=-.6:step:1; Zrange=.45:step:1;
[X,Y,Z] = meshgrid(Xrange,Yrange,Zrange);
tic;
F=BSpline3DF(P,X(:),Y(:),Z(:));
Time_REC=toc
W=reshape(F,size(X));
%Isosurface from MatLab:--------------------
%W=postprocess(W,X,Y,Z,x,y,z,5);

%Isosurface from MatLab:--------------------
%W=postprocess(W,X,Y,Z,x,y,z,5);
stop=0; Level=0.0; vw=2; [-58 48]; [90 0];
ch=[.5 .6 .8]; %ch=[.9 .8 .1];
while ~stop    
    hh=figure; set(hh,'Position',[900 200 700 800]); 
    hold on; axis equal off; view(vw); camlight;
    p = patch(isosurface(X, Y, Z, W, Level));
    isonormals(X,Y,Z,W, p); 
    set(p, 'FaceColor', ch, 'EdgeColor', 'none'); lighting phong;
    set(gca, 'LooseInset', [0,0,0,0]);axis equal off
    if ShowOtherLevels
        Level=input('Which iso-surface to show? (exit: 0) ');
    end
    if (Level==0); stop=1; end
end

%My own function without rendering:--------------------
if run_mine
    figure
    cntr=0;
    for i=2:size(W,1)-1
        levelNo(i)=cntr;
        if cntr==0
            firstInd=i;
        end
        for j=2:size(W,2)-1
            for k=2:size(W,3)-1
                if (W(i,j,k)*W(i-1,j,k)<0)||(W(i,j,k)*W(i,j-1,k)<0)||(W(i,j,k)*W(i,j,k-1)<0)
                    if (W(i,j,k-1)*W(i,j,k+1)<0)
                        cntr=cntr+1;
                        %scatter3(X(i,j,k),Y(i,j,k),Z(i,j,k),'r.')
                        x(cntr)=X(i,j,k); y(cntr)=Y(i,j,k); z(cntr)=Z(i,j,k);
                        %myPatch(c,step,x(cntr),y(cntr),z(cntr));
                    end
                end
            end
        end
    end
end
%box on;

function F=BSpline3DF(P,x,y,z)
%%
n=numel(x); N=size(P,1); %N=size(P,2); O=size(P,3); 
stepX=1/(N-3);
%MM=zeros(n,N^3);
F=zeros(n,1);

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
            %Matlab standard order: layer-by-layer, column-by...           
            Col=N^2*(k+kk-1)+N*(j+jj-1)+(i+ii); 
            F=F+P(Col).*bu(:,ii+1).*bv(:,jj+1).*bw(:,kk+1);
        end
    end
end

%removing zeros from the head:
function myTriangle(normal,P1,P2,P3)
hold on;
% normal=cross(P1-P2,P1-P3);
% if norm(normal)
%     normal=abs(normal)/norm(normal);
% else
%     normal=[0 0 0];
% end
col=abs(normal*[1 0 0]')*[1 0 0]
%normal*[1 0 0]'*[1 0 0]
%col=col/norm(col);
if isnan(col)
    col=[1 0 0];
end
X=[P1(1),P2(1),P3(1)]; Y=[P1(2),P2(2),P3(2)]; Z=[P1(3),P2(3),P3(3)];
patch(X,Y,Z,abs(col));



function myPatch(c,step,xi,yi,zi)
%first of all, we should find 4 points on the corners:
[f, fx,fy,fz,fxx,fxy,fxz,fyy,fyz,fzz]=Poly(deg,c,xi,yi,zi);
%I hope fz is nonzero!
DeltaZ1=(-fx/fz-fy/fz)*step/2;
DeltaZ2=(+fx/fz-fy/fz)*step/2;
swapxz=0;
if abs(DeltaZ1)+abs(DeltaZ2)>8*step
    DeltaX1=(-fz/fx-fy/fx)*step/2;
    DeltaX2=(+fz/fx-fy/fx)*step/2;
    %swap x & z:
    swapxz=1;
    tt=xi; xi=zi; zi=tt;
    tt=fx; fx=fz; fz=tt;
    DeltaZ1=DeltaX1; DeltaZ2=DeltaX2;    
end
P=[xi,yi,zi];
normal=[fx, fy, fz]; normal=normal/norm(normal);
%quiver3(P(1),P(2),P(3),n(1),n(2),n(3),'g','LineWidth',2);
P1=[xi+step/2, yi+step/2,zi+DeltaZ1];
P2=[xi-step/2, yi+step/2,zi+DeltaZ2];
P3=[xi-step/2, yi-step/2,zi-DeltaZ1];
P4=[xi+step/2, yi-step/2,zi-DeltaZ2];
if swapxz
    tt=P(1); P(1)=P(3); P(3)=tt;
    tt=P1(1); P1(1)=P1(3); P1(3)=tt;
    tt=P2(1); P2(1)=P2(3); P2(3)=tt;
    tt=P3(1); P3(1)=P3(3); P3(3)=tt;
    tt=P4(1); P4(1)=P4(3); P4(3)=tt;
end
%the extreme case:
%draw all 4 triangles:
myTriangle(normal,P,P1,P2);
myTriangle(normal,P,P2,P3);
myTriangle(normal,P,P3,P4);
myTriangle(normal,P,P4,P1);
% scatter3(P(1),P(2),P(3),120,'filled')
% scatter3(P1(1),P1(2),P1(3),'filled')
% scatter3(P2(1),P2(2),P2(3),'filled')
% scatter3(P3(1),P3(2),P3(3),'filled')
% scatter3(P4(1),P4(2),P4(3),'filled')