function [in out,Normal]=NewOffset(r,pnt, tri)
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
%Input: pnt: point cloud; tri: triangulation; r: offset distance.
%Output: in: inner offset; out: outer offset; Normal: normal vectors.

if size(pnt,2)>3
    pnt=pnt'; %p will be 2xn
end

tri=tri+1;
m=size(pnt,1); n=size(tri,1);
tri_normal=tri*0;
pnt_normal=pnt*0;
associateTri=zeros(n,41); %each vertex is associated to at most 40 triangles!
%The first cell shows the number of associated Triangles
%--------------------------------------------------------------------------
w1=waitbar(0,'Constructing the Offset');
for i=1:n
    if (mod(i,200)==0)
        waitbar(i/(n+m),w1);
    end
    ind=tri(i,:);
    P=pnt(ind(1:3),1:3);     
    %myTriangle(P(1,:), P(2,:), P(3,:));
    tri_normal(i,:)=cross(P(2,:)-P(1,:),P(3,:)-P(2,:));    
    tri_normal(i,:)=tri_normal(i,:)/norm(tri_normal(i,:));    
end
for i=1:n %traverse all the triangles:   
    for k=1:3
        j=tri(i,k);%the associated vertex
        associateTri(j,1)=associateTri(j,1)+1;
        associateTri(j,associateTri(j,1)+1)=i;
    end
end        

for j=1:m %this is a faster version
    if (mod(j,200)==0)
        waitbar((n+j)/(m+n),w1);
    end
    current_normal=[0 0 0];
    NT=associateTri(j,1);
    for i=associateTri(j,2:NT+1)            
        current_normal=current_normal+tri_normal(i,:);
    end           
    current_normal=current_normal/NT;
    pnt_normal(j,:)=current_normal;
end
in= pnt+r*pnt_normal;
out=pnt-r*pnt_normal;
Normal=pnt_normal;
for i=1:size(Normal,1)
    Normal(i,:)=Normal(i,:)/norm(Normal(i,:));
end
close(w1)

% scatter3(in(:,1),in(:,2),in(:,3),'r','filled');  hold on
% scatter3(out(:,1),out(:,2),out(:,3),'g','filled');
%--------------------------------------------------------------------------