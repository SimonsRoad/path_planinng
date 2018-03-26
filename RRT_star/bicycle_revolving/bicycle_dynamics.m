function xdot=bicycle_dynamics(x,u)
% x=[x y theta v steer]  u=[rear wheel linear accecation (vdot) , steering acceleration ]
% L is distance btx wheels
L=1;

% %% input = [accecleration]
% xdot=[x(4)*cos(x(3)) ; x(4)*sin(x(3)) ;tan(x(5)*x(4))/L; 0 ;0 ]+ ...
%     [0 0 0 1 0]'*u(1)+[0 0 0 0 1]'*u(2);
%% input=[velocity and steering angle]
xdot=[u(1)*cos(x(3));u(1)*sin(x(3));tan(u(2)*u(1))/L];

end