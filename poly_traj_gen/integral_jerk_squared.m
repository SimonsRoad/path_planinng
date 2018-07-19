function Q_j=integral_jerk_squared(n)
% poly order = n
% integral from 0 to 1 

Q_j=zeros(n+1);            
for i=4:n+1
    for j=4:n+1
        if i==4 && j==4
            Q_j(i,j)=(i-1)*(i-2)*(i-3)*(j-1)*(j-2)*(j-3);
        else
            Q_j(i,j)=(i-1)*(i-2)*(i-3)*(j-1)*(j-2)*(j-3)/(i+j-7);
        end
    end
end

end