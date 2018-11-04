% specifying variable properties
nC = 10; %Number of Continuous Variables
nI = 10; %Number of Integer Variables
nB = 10; %Number of Binary Variables

% Build xtype vector
xtype = [repmat('C',1,nC),repmat('I',1,nI),repmat('B',1,nB)];

% Objective
H = [1 -1; -1  2];          %Objective Function (min 0.5x'Hx + f'x)
f = -[2 6]';                

% Constraints
A = [1,1; -1,2; 2, 1];      %Linear Inequality Constraints (Ax <= b)
b = [2;2;3];    
lb = [0;0];                 %Bounds on x (lb <= x)

xtype = 'IC';

% Create OPTI Object
Opt = opti('qp',H,f,'ineq',A,b,'lb',lb,'xtype',xtype);

% Solve the MIQP problem
[x,fval,exitflag,info] = solve(Opt)