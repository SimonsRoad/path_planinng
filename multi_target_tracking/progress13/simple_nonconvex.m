d_des = 3;
xc = 1; yc =1;
f = @(x,y) (abs(x-xc) + abs(y-yc) - d_des).^2;
[xs,ys]=meshgrid(-3:0.2:5,-3:0.2:5);
% f(xs,ys)
figure
surf(xs,ys,f(xs,ys))
hold on
str = '$\mathbf{x}_{g,h} -\mathbf{x}_{h}_{1} - d_{des})^{2}$';
% title(str,'Interpreter','latex')
title('$f(\mathbf{x}_{h}) = (|\mathbf{x}_{g,h} - \mathbf{x}_{h}|_{1} - d_{des})^{2}$','Interpreter','latex')

colorbar