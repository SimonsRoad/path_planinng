function answer=iswithin(value,ranges)
% value= D x 1 
% ranges = D x 2
answer=1;
d=length(value);
    for i=1:d
        upper=ranges(i,2);
        lower=ranges(i,1);
        answer=answer && (value(i)<upper && value(i)>lower);
     end



end