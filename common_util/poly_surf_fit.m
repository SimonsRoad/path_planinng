function p=poly_surf_fit(x,y,z,nx,ny)
% this function fit the give data point with polynomial surface 
% with f(x,y)=sigma( p_ij * x^i y^j )_{i=0... nx, j=0... ny}
% note: i+j<=max(nx,ny)
z=double(z);
if nx <ny 
    tmp=x; tmp_n=nx;
    x=y; nx=ny;
    y=tmp; ny=tmp_n;
end

%so.. now nx > ny 

% normalizing factor 
lx=max(abs(x));
ly=max(abs(y));

% possible (i,j)
order_pair=[];
for order=0:nx
    for i=order:-1:0
        if order-i <= ny            
        order_pair=[order_pair ; [i order-i]];
        end
    end
end

%% qp solver

A=zeros(length(order_pair));
b=zeros(length(order_pair),1);

for k=1:length(z)
    X_k=zeros(length(order_pair),1); %normalized
    for i=1:length(order_pair)
        X_k(i)=(x(k))^(order_pair(i,1))*(y(k))^(order_pair(i,2));    
    end
    A=A+X_k*X_k';
    b=b+-2*X_k*z(k);
end

p=quadprog(2*A,b);


%% direct calculation 
% % normalize matrix 
% M=zeros(length(order_pair));
% M_inv=M;
% A=M;
% b=zeros(length(order_pair),1);
% for i=1:length(order_pair)
%     M(i,i)=lx^(order_pair(i,1))*ly^(order_pair(i,2));
%     M_inv(i,i)=1/M(i,i);    
% end
% 
% % to solve (NAN')p=Nb
% 
% for k=1:length(z)
%     X_k=zeros(length(order_pair),1); %normalized
%     for i=1:length(order_pair)
%         X_k(i)=(x(k)/lx)^(order_pair(i,1))*(y(k)/ly)^(order_pair(i,2));    
%     end
%     A=A+X_k*X_k';
%     b=b+X_k*z(k);
% end
% 
% p=M_inv*A\b;

end






