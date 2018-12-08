function [proj,depth]=proj_image(f,p,R,pts)
    % proj pt into image plane of (f,p,Rwc)
    % pt : 3 x N / p : 3 x 1
    % proj : 2 x N
    
    N = size(pts,2);
    proj = zeros(2,N);
    depth = zeros(1,N);
    
    for  i = 1:N
        pnt = pts(:,i);
        xc=R'*(pnt-p);
        proj(:,i)=f/xc(1)*[xc(2); xc(3)];       
        depth(i) = xc(1); % we should consider the depth for overlapping each object 
    end       
end
    
