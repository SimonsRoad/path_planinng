function [check,A,b]=is_in_blind(target1,target2,FOV,x_pnts)
    % FOV constriant evalutation 
    % x_pnts : set evalutation pnt  (2 x N)      
    % this is very relaxed evaluation ()
    
    target1 = reshape(target1,2,1);
    target2 = reshape(target2,2,1);
    x_pnts = reshape(x_pnts,2,[]);
    v = target2 -target1 ;
    l = norm(v);
    mid_pnt = (target1 + target2)/2;
        
    rot90 = [0 -1 ; 1  0]; 
    rot90_inv = [0 1;-1 0];   
    
    v1 = rot90 * v ; v1 = v1/norm(v1);
    v2 = rot90_inv * v; v2 = v2/norm(v2);
    
    p1 = mid_pnt + l/2/tan(FOV/2) * v1;
    p2 = mid_pnt + l/2/tan(FOV/2) *v2 ;
    
    % the representation of the region 

     A = zeros(4,2);
     b = zeros(4,1);
     
     A(1,:) = rot90 * (target2 - p1); b(1) = A(1,:) * p1;
     A(2,:) = rot90 * (p1 - target1); b(2) = A(2,:) * p1;     
     A(3,:) = rot90_inv * (p2 - target1); b(3) = A(3,:) * p2;
     A(4,:)  =rot90_inv * (target2 - p2); b(4) = A(4,:) * p2;
             
   check = zeros(1,size(x_pnts,2));     
    for col = 1:size(x_pnts,2) 
        check(col) =(sum(A*x_pnts(:,col) <= b) == 4);
    end
  
end