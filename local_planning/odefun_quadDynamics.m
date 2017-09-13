
function[dx, xd, f,M] = odefun_quadDynamics(t,x,data)
% Extracing parameters
% --------------------
% Dynamics of quadrotor suspended with load Constants
mQ = data.params.mQ;
J = data.params.J;
g = data.params.g;
e1 = data.params.e1;
e2 = data.params.e2;
e3 = data.params.e3;
p=data.params.p;
n=data.params.n;
tspan=data.params.tspan;
% fetching desired states
% -----------------------
[trajd] = traj(data.params.p,t,n,tspan(1),tspan(2));
% [trajd] = get_flats(t);

xQd = trajd.x;
vQd = trajd.dx;
aQd = trajd.d2x;

Omegad = zeros(3,1);



% Extracing states
% ----------------
xQ = x(1:3);
vQ = x(4:6);
R = reshape(x(7:15),3,3);
Omega = x(16:18);
b3 = R(:,3);
dx = [];

    % CONTROL
    % ------
    % Position Control
    eQ = xQ - xQd;
    deQ = vQ - vQd;

    epsilon_bar = 0.8 ;
    kp_xy = 0.3/epsilon_bar^2 ; kd_xy = 0.6/epsilon_bar ;
    k1 = diag([kp_xy kp_xy 2]) ; k2 = diag([kd_xy kd_xy 1.5]) ;

    k1 =  20*diag([4, 5 ,6]);
    k2 = 0.5*diag([4, 4, 6]);
    A = (-k1*eQ - k2*deQ + (mQ)*(aQd + g*e3));
    normA = norm(A);
    b3c = A/norm(A);

    f = vec_dot(A,b3);
    
    % Attitude Control
    b1d=e1;
    b1c = -vec_cross(b3c,vec_cross(b3c,b1d));
    b1c = b1c/norm(vec_cross(b3c,b1d));
    Rc = [b1c vec_cross(b3c,b1c) b3c];
    Rd = Rc;

    if(norm(Rd'*Rd-eye(3)) > 1e-2)
        disp('Error in R') ; keyboard ;
    end

    kR = 50 ; kOm = 0.5;
    epsilon = 0.1 ; %.5 ; %0.01 ;

    err_R = 1/2 * vee_map(Rd'*R - R'*Rd) ;
    err_Om = Omega - R'*Rd*Omegad ;
    M = -kR*err_R - kOm*err_Om+skew(Omega)*(J*Omega) ;
    
    % Equations of Motion
    % -------------------
    xQ_dot = vQ;
    vQ_dot = -g*e3 + (f/mQ)*R*e3;
    
    
    R_dot = R*hat_map(Omega) ;
    Omega_dot = J\( -vec_cross(Omega, J*Omega) + M ) ;
    
    
% Computing xd
% ------------
xd = [xQd; vQd ;aQd];
xd = [xd;reshape(Rd, 9,1);Omegad];

% Computing dx
%-------------
dx = [xQ_dot;
      vQ_dot;
      reshape(R_dot, 9,1) ;
      Omega_dot;];

if nargout <= 1
   fprintf('Sim time %0.4f seconds \n',t);
end
    
end