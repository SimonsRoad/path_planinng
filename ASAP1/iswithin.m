function answer=iswithin(value,ranges)
% value= D x N
% ranges = D x 2 
% if any one point of N value points is included in ranges, it returns true 
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