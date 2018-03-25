function res=fit_poly_fun(X_eval,sf)
    % this function computes the result of multi-dim poly fitting 
    % sf = sfit object 
    % Xval= [x_val y_val]
    coeffi_names=coeffnames(sf);
    coeffi_values=coeffvalues(sf);
    res=0;
    for i=1:length(coeffi_names)
        cur_name=coeffi_names{i};
        nx=str2num(cur_name(2)); ny=str2num(cur_name(3));
        res=res+ coeffi_values(i) * (X_eval(1)^nx)*(X_eval(2)^ny);
    end
end