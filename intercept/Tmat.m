function T_mat=Tmat(t,k,n)
%p73
% obtaining T_mat matrix of k derivative of nth order polynomial 
% that is, x(t)=T_mat*p of dx(t)=T_mat2*p.....

switch k
    
    case 0
        
        T_mat=eye(3);
        for i=1:n
            T_mat=[T_mat t^i*eye(3)];    
        end

    case 1
        
        T_mat=zeros(3);
     
        for i=1:n
            T_mat=[T_mat i*t^(i-1)*eye(3) ];    
        end
        
    case 2
        T_mat=[zeros(3) zeros(3)];
        for i=2:n
            T_mat=[T_mat (i-1)*i*t^(i-2)*eye(3)];
        end


end
            
         
end