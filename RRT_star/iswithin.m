function answer=iswithin(value,ranges)
% value= D x 1 
% ranges = D x 2
[d,n]=size(value);
answer=0;
for j=1:n
    answer_sub=1;
    for i=1:d
        upper=ranges(i,2);
        lower=ranges(i,1);
        answer_sub=answer_sub && (value(i,j)<upper && value(i,j)>lower);
    end
    answer=answer_sub || answer;
end

end