function res=atan3(Y,X)
    res=atan2(Y,X);
    if res<0
        res=2*pi+res;
    end


end