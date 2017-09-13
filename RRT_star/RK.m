function x_next=RK(x,u,dt,f)
% this is Runge-Kunta numerical integration method function 
% usage eg : f input=@sin  
% dynamics is f in xdot=f(x,u)
% see note p81
xdot= f(x+dt/2*f(x,u),u);
x2dot=f(x+dt/2*xdot,u);
x3dot=f(x+dt/2*x2dot,u);
x_next=x+dt/6*(f(x,u)+2*xdot+2*x2dot+x3dot);
end