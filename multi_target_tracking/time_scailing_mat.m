function D = time_scailing_mat(t1,t2,poly_order)
    d=t2-t1;
    D=zeros(poly_order);
    
    for i=1:poly_order+1
        D(i,i)=d^(i-1);
    end
    
    


end