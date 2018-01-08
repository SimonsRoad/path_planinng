function t_vec=t_vector(t,n_d)
% this function generate vector t s.t p(t)=p'*t
% n_d : number of derivative 
% t : time value 

global n % number of order of polynomial 

t_vec=zeros(n+1,1);
for r=(n_d): n
    t_vec(r+1)=factorial(r)/factorial(r-n_d)*t^(r-n_d);
end

