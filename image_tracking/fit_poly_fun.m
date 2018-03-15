function res=fit_poly_fun(X_eval,p,nx,ny)
    % this function computes the result of multi-dim poly fitting 
    % X_eval=[x y]
    count=1;
    res=0;
    for n=0:max(nx,ny) % but nx>ny assumed 
       % generate combination [nx_cur,ny_cur] whose sum nx_cur+ny_cur=n 
        for nx_cur=linspace(n,0,n+1)
            if count > length(p)
                return 
                
            end
            res=res+ p(count)*X_eval(1)^(nx_cur)*X_eval(2)^(n-nx_cur);
            count=count+1;
            
        end

    end
end