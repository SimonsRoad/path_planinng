function [A,b]=get_ineq_matrix(g,v1,v2)
    % this function returns A,B s.t Ax <= b corresponds to the region between two vectors g+v1, g+v2 
    v1 = v1/norm(v1);
    v2 = v2/norm(v2);
    pk = g + (v1 + v2)/2; % inspection point 
   
    v1_norm = [-v1(2); v1(1)];
    v2_norm = [-v2(2); v2(1)];
    n = [0,1]; m = [0,1]; % we need to select correct pair of sign 
    correct_sign = [0,0];
    for i = n 
        for j = m 
            if ( (-1)^i*v1_norm'*pk <= (-1)^i*v1_norm'*g) && ( (-1)^j*v2_norm'*pk <= (-1)^j*v2_norm'*g)
                correct_sign = [i,j];
            end
        end
    end

    A = [(-1)^(correct_sign(1))*v1_norm' ; (-1)^(correct_sign(2))*v2_norm' ]; b = [(-1)^correct_sign(1)*v1_norm'*g;(-1)^correct_sign(2)*v2_norm'*g];
    
end