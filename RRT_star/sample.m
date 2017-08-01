function x_rand=sample(problem)
    d=problem.dim;
    x_rand=zeros(1,d);
    for i=1:d
        ith_range=problem.range(i,:);
        x_rand(i)=(ith_range(2)-ith_range(1))*rand+ith_range(1);
    end
end
