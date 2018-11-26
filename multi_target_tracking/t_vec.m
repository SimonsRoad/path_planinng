function vec=t_vec(n,t,n_diff)
    % this computes time vector.
    % n: order of poly
    % t: eval point 
    % n_diff : diff order 

    vec=zeros(n+1,1);
    switch n_diff
        case 0
            for i=1:n+1
                vec(i)=t^(i-1);
            end            
        case 1
            for i=2:n+1
                vec(i)=(i-1)*t^(i-2);
            end

        case 2
            for i=3:n+1
                vec(i)=(i-1)*(i-2)*t^(i-3);
            end                       
    end        
end
