function cost=cost_metric(x1,x2)
global kR kx kv kw 

[xl1,vl1,Rl1,wl1]=x2s(x1);
[xl2,vl2,Rl2,wl2]=x2s(x2);

cost=kR*(norm(vex(Rl1'*Rl2-Rl2'*Rl1)))+kx*norm(xl1-xl2)^2+kv*norm(vl1-vl2)^2+kw*norm(wl1-Rl1'*Rl2*wl2);

end