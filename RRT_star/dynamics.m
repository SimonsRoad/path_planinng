function xdot=dynamics(x,u)
global l
xdot=[x(7:end)
    u(1)*(cos(x(4))*sin(x(5))*cos(x(6))+sin(x(4))*sin(x(6))) 
    u(1)*(cos(x(4))*sin(x(5))*sin(x(6))-sin(x(4))*sin(x(6)))
    u(1)*cos(x(4))*cos(x(5))-9.81
    u(2)*l
    u(3)*l
    u(4)
    ];
end