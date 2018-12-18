function output=dynamics_pd(input)

g = 9.81;
l = 0.4;
m = 1;

u1 = input(1);
u2 = input(2);
u3 = input(3);
u4 = input(4);

x = input(5); 
y = input(6) ; 
z = input(7) ; 

phi = input(8);
theta = input(9);
psi = input(10);

xdot = input(11) ; 
ydot = input(12) ; 
zdot = input(13) ; 

phidot = input(14) ; 
thetadot = input(15) ; 
psidot = input(16) ;


xddot = u1*(cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi))/m;
yddot = u1*(cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi))/m;
zddot = u1*cos(phi)*cos(theta)/m-g;

phiddot = u2*l;
thetaddot = u3*l;
psiddot = u4;

output=[xdot ydot zdot phidot thetadot psidot xddot yddot zddot phiddot thetaddot psiddot];