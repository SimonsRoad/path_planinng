% this script test the following problem 

% min | |x-xc| + |y-yc| - d_des |
%  L<= x,y <= U 
% variables : X+,  X-,  Y+, Y-,  zx, zy S  
%% LP formulation 
% 0.014
ub = 1;
lb = -1; 
% lb <= x <= ub , lb <= y <= ub
N_var = 7;
d_des = 1;
xc = -0.5;
yc = -0.5;

Lx = lb - xc; Ux = ub - xc;
Ly = lb - yc; Uy = ub - yc;

% var : X+, X-, Y+, Y-, Ax, Ay, D  ++ X, Y  

Aineq = [
    -1 0 0 0 max(0,Lx) 0 0; 
    1 0 0 0 -max(0,Ux) 0 0;
    0 -1 0 0 -max(0,-Ux) 0 0;
    0 1 0 0 max(0,-Lx) 0 0 ;
    0 0 -1 0 0 max(0,Ly) 0;
    0 0 1 0 0 -max(0,Uy) 0;
    0 0 0 -1 0 -max(0,-Uy) 0;
    0 0 0 1 0 max(0,-Ly)  0;
    -1 -1 -1 -1 0 0 -1;
    1 1 1 1 0 0 -1;
    0 0 0 0 -1 0 0;
    0 0 0 0 1 0 0 ;
    0 0 0 0 0 -1 0;
    0 0 0 0 0 1 0 
    ];

bineq = [0 0 -max(0,-Ux) max(0,-Lx) 0 0 -max(0,-Uy) max(0,-Ly) -d_des d_des -((abs(Lx)+Lx)/(2*abs(Lx))) ((abs(Ux)+Ux)/(2*abs(Ux))) -((abs(Ly)+Ly)/(2*abs(Ly))) ((abs(Uy)+Uy)/(2*abs(Uy)))]';

Aineq = [Aineq zeros(size(Aineq,1),2)];
Aeq = [-1 1 0 0 0 0 0 1 0;
            0 0 -1 1 0 0 0 0 1];
beq = [xc ; yc];


f = [0 0 0 0 0 0 1 0 0];
lbs = [-inf -inf -inf -inf 0 0 -inf lb lb];
ubs = [inf inf inf inf 1 1 inf ub ub];
intcon = 5:6;

tic 
sol = intlinprog(f,intcon,Aineq,bineq,Aeq,beq);
toc
x = xc + sol(1) - sol(2);
y = yc + sol(3) - sol(4);


%% nonlinear optimization 
% 0.14
f = @(x) abs(abs(x(1)) + abs(x(2)) - d_des);
x0 = [0 0];
tic
sol = fmincon(f,x0,[],[],[2 1],[2]);
toc















 




