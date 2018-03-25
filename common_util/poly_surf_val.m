function z=poly_surf_val(X,p,nx,ny)
x=X(1); y=X(2);
if nx <ny 
    tmp=x; tmp_n=nx;
    x=y; nx=ny;
    y=tmp; ny=tmp_n;
end

%so.. now nx > ny 

% possible (i,j)
order_pair=[];
for order=0:nx
    for i=order:-1:0
        if order-i <= ny            
        order_pair=[order_pair ; [i order-i]];
        end
    end
end

X=zeros(length(order_pair),1);
for i=1:length(order_pair)
    X(i)=x^(order_pair(i,1))*y^(order_pair(i,2));
end

z=p'*X;

end
















