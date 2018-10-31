% this script test the following problem 

% min | |x-xc| + |y-yc| - d_des |
%  L<= x,y <= U 
% variables : X+,  X-,  Y+, Y-,  zx, zy S  
%% LP formulation 
% 0.014
ub = 1;
lb = -1; 
N_var = 7;
d_des = 1;
xc = 2;
yc = 2;

Lx=max(-lb +xc,0 ); Ux = max(ub -xc , 0);
Ly = max(-lb + yc,0); Uy = max(ub - yc,0);

Aineq = [
    -1 0 0 0 0 0 0; 
    1 0 0 0 -Ux 0 0;
    0 -1 0 0 0 0 0;
    0 1 0 0 Lx 0 0 ;
    0 0 -1 0 0 0 0;
    0 0 1 0 0 -Uy 0;
    0 0 0 -1 0 0 0;
    0 0 0 1 0 Ly 0;
    -1 -1 -1 -1 0 0 -1;
    1 1 1 1 0 0 -1;
    ];

bineq = [0 0 0 -L 0 0 0 -L -d_des d_des]';
% Aeq = [2 -2 1 -1 0 0 0 ];
% beq = 2;



Aeq = [];
beq = [];

f = [0 0 0 0 0 0 1];
lbs = [-inf -inf -inf -inf 0 0 -inf];
ubs = [inf inf inf inf 1 1 inf];
intcon = 5:6;

tic 
sol = intlinprog(f,intcon,Aineq,bineq,Aeq,beq,lbs,ubs);
x = xc + sol(1) - sol(2);
y = yc + sol(3) - sol(4);

toc

%% nonlinear optimization 
% 0.14
f = @(x) abs(abs(x(1)) + abs(x(2)) - d_des);
x0 = [0 0];
tic
sol = fmincon(f,x0,[],[],[2 1],[2]);
toc















 




