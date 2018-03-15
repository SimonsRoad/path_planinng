function [c,ceq]=nonlcon(X,coeff)
    % coeff : coeff of fitted polynomial 
    % X = [r azim elev] (3 x 1)
    global nx ny
    r=X(1); azim=X(2); elev=X(3);
    c=r-(fit_poly_fun(X(2:3),coeff,nx,ny));
    ceq=[];
end