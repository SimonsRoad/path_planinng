function res=col_norm(mat)
res=[];

for i=1: length(mat)
    res=[res norm(mat(:,i))];
end


end