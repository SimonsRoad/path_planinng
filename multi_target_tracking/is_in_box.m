function res = is_in_box(vk,xg,x_range,y_range)
    %% this function investigate whether a half line xg + vk * t (t>0) has crossed
    % the box defined by the x_range,y_range
    % return true if the intersection point is on the box or false if not
    vk = reshape(vk,2,1);
    vk_conj = [vk(2);vk(1)];
    xg = reshape(xg,2,1);
    x_range = reshape(x_range,2,1);
    y_range = reshape(y_range,2,1);
    xl = x_range(1); xu = x_range(2);
    yl = y_range(1); yu = y_range(2);
    
  
    res = false;
    
    %% let's construct a linear equation form 
     
    % cross point with x = xl 
    A = [vk_conj' ; 1 0 ]; b =[vk_conj'*xg ; xl]; 
    inter_pnt = inv(A)*b;
    if (inter_pnt(2) < yu) && (inter_pnt(2) > yl) && ((inter_pnt(2)-xg(2))/vk(2) >0)
        res = true;
        return;
    end
    
    % cross point with x = xu
    A = [vk_conj' ; 1 0 ]; b =[vk_conj'*xg ; xu]; 
    inter_pnt = inv(A)*b;
    if (inter_pnt(2) < yu) && (inter_pnt(2) > yl) && ((inter_pnt(2)-xg(2))/vk(2) >0)
        res = true;
        return;
    end
    
    % cross point with y = yl 
    A = [vk_conj' ; 0 1 ]; b =[vk_conj'*xg ; yl]; 
    inter_pnt = inv(A)*b;
    if (inter_pnt(1) < xu) && (inter_pnt(1) > xl) && ((inter_pnt(2)-xg(2))/vk(2) >0)
        res = true;
        return;
    end
       
    % cross point with y = yu     
    A = [vk_conj' ; 0 1 ]; b =[vk_conj'*xg ; yu]; 
    inter_pnt = inv(A)*b;
    if (inter_pnt(1) < xu) && (inter_pnt(1) > xl) && ((inter_pnt(2)-xg(2))/vk(2) >0)
        res = true;
        return;
    end
    
   
end