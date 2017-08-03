function x_rand=sample(problem)
    d=problem.dim;
    nobs=length(problem.obs);
    isobs=1;
    
    
    while isobs
        x_rand=zeros(d,1);
        
        for i=1:d
            ith_range=problem.range(i,:);
            x_rand(i)=(ith_range(2)-ith_range(1))*rand+ith_range(1);
        end
        % obstacle check 
        isobs_sub=0;
        for i=1:nobs
        isobs_sub=isobs_sub || iswithin(x_rand,problem.obs{i});
        end
        isobs=isobs_sub;
        
    end
    
    
    
end
