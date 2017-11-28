function [X0s, Y0s]=initial_guess_traj(x0,xN,N)

X0=linspace(x0(1),xN(1),N); Y0=linspace(x0(2),xN(2),N);
% plot(X0,Y0,'b-')
X0s=X0; Y0s=Y0;


% mutiple initial guess for global search 

xc=(x0+xN)/2;
v=(xN-x0); v=[v(2) -v(1)]/norm(v);
pseudo_t=[0 0.5 1]; % this is for polyfit dummy input variable 

d=[1/4 1/2 -1/4 -1/2];

for i=1:length(d)
    xm=xc+v'*d(i)*norm(xN-x0);
    
    px=polyfit(pseudo_t,[x0(1) xm(1) xN(1)],2);
    py=polyfit(pseudo_t,[x0(2) xm(2) xN(2)],2);
    X0=polyval(px,linspace(0,1,N));
    Y0=polyval(py,linspace(0,1,N));
    X0s=[X0s; X0]; Y0s=[Y0s ;Y0];
end



end