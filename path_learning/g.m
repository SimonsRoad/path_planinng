function [C,Ceq]=g(x)

g1=@(x) x(1)*(1-log(x(1)))-1;
g2=@(x) -3*x(1)+2*x(2)^2-1;
g3=@(x) -x(1);


C=[g1(x);g2(x);g3(x)];
Ceq=[];




end