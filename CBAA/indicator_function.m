%% Indicater function 
function res_bool=indicator_function(ci,yi)
    len=length(ci);
    res_bool=zeros(1,len); % 1 x D
    for j=1:len
        res_bool(j)=ci(j)>yi(j);
    end
end


