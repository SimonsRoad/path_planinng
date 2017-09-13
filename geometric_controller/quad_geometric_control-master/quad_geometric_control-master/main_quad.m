function[varargout] =  main_quad(varargin)
% Geometric control of Quadrotor on SE(3)
% http://www.math.ucsd.edu/~mleok/pdf/LeLeMc2010_quadrotor.pdf
%% TRAJECTORY GENERATION

trajectory_gen

%% INITIALZING WORKSPACE
% ======================
% Clear workspace
% ---------------
% clear; 
% clc;

% Add Paths
% ----------
% Adding path to 'Geometric Control Toolbox'
addpath('./Geometry-Toolbox/');


%% INITIALZING PARAMETERS
% ======================
% System constants and parameters
data.params.mQ = 0.2 ;
data.params.J = diag([0.557, 0.557, 1.05]*10e-3);
data.params.g = 9.81 ;
data.params.e1 = [1;0;0] ;
data.params.e2 = [0;1;0] ;
data.params.e3 = [0;0;1] ;
data.params.p=p; % coefficients of polynomial  
data.params.n=n; % order of polynomial  

%% INTIALIZING - INTIAL CONDITIONS
% ================================
% Zero Position 
% -------------
xQ0 = [];
vQ0 = zeros(3,1);

R0 = RPYtoRot_ZXY(010*pi/180,0*pi/180, 0*pi/180) ;
Omega0 = zeros(3,1);

xQ0 = x0-ones(3,1);
vQ0 = dx0_real;
% 
R0=[1 0 0; 0 1 0 ; 0 0 1];
Omega0 = zeros(3,1);


% Zero Initial Error- Configuration
% ---------------------------------
% [trajd0] = get_nom_traj(data.params,get_flats(0));
% xQ0 = trajd0.xQ;
% vQ0 = trajd0.vQ;
% 
% R0 = trajd0.R;
% Omega0 = trajd0.Omega;

% state - structure
% -----------------
% [xL; vL; R; Omega]
% setting up x0 (initial state)
% -----------------------------
<<<<<<< HEAD
x0 = [xQ0; vQ0; reshape(R0,9,1); Omega0 ];
%traj_plot([0 20])
=======
x_init = [xQ0; vQ0; reshape(R0,9,1); Omega0 ];
>>>>>>> 9922288ccf8eb667e958caa9fb4b9a3e1a4abe4e

%% SIMULATION
% ==========
disp('Simulating...') ;
odeopts = odeset('RelTol', 1e-8, 'AbsTol', 1e-9) ;
% odeopts = [] ;
tspan=[t0_real tf_real];
data.params.tspan=tspan;
[t, x] = ode15s(@odefun_quadDynamics, tspan, x_init, odeopts, data) ;

% Computing Various Quantities
disp('Computing...') ;
ind = round(linspace(1, length(t), round(1*length(t)))) ;
% ind = 0:length(t);
for i = ind
   [~,xd_,f_,M_] =  odefun_quadDynamics(t(i),x(i,:)',data);
   xd(i,:) = xd_';
   psi_exL(i) = norm(x(i,1:3)-xd(i,1:3));
   psi_evL(i) = norm(x(i,4:6)-xd(i,4:6));
   f(i,1)= f_;
   M(i,:)= M_';
end


%% PLOTS
% =====
    figure;
    subplot(2,2,1);
    plot(t(ind),x(ind,1),'-g',t(ind),xd(ind,1),':r');
    grid on; title('x');legend('x','x_d');%axis equal;
    xlabel('time');ylabel('x [m]');
    subplot(2,2,2);
    plot(t(ind),x(ind,2),'-g',t(ind),xd(ind,2),':r');
    grid on; title('y');legend('y','y_d');%axis equal;
    xlabel('time');ylabel('y [m]');
    subplot(2,2,3);
    plot(t(ind),x(ind,3),'-g',t(ind),xd(ind,3),':r');
    grid on; title('z');legend('z','z_d');%axis equal;
    xlabel('time');ylabel('z [m]');
    subplot(2,2,4);
<<<<<<< HEAD
%     plot3(x(ind,1),x(ind,2),x(ind,3),'-g',xd(ind,1),xd(ind,2),xd(ind,3),':r');
%     grid on; title('trajectory');legend('traj','traj_d');%axis equal;
%     xlabel('x-axis');ylabel('y-axis');zlabel('z-axis');

=======
    
    plot3(x(ind,1),x(ind,2),x(ind,3),'-g',xd(ind,1),xd(ind,2),xd(ind,3),':r');
    
    grid on; title('trajectory');legend('traj','traj_d');%axis equal;
    xlabel('x-axis');ylabel('y-axis');zlabel('z-axis');
>>>>>>> 9922288ccf8eb667e958caa9fb4b9a3e1a4abe4e

    figure;
    subplot(2,1,1);
    plot(t(ind),psi_exL(ind));
    grid on; title('position error');legend('psi-exL');
    subplot(2,1,2);
    plot(t(ind),psi_evL(ind));
    grid on; title('velocity error');legend('psi-evL');
<<<<<<< HEAD
% ANIMATION 
    figure()
    hold on

    frame_nb=10;
    step=floor(length(t)/frame_nb);
    traj_plot([0 20]);
    
    for i=1:step:length(t)
    R=reshape(x(i,7:15),3,3);
    draw_drone2(R,x(i,1:3),1000,500,2)
              
    end
    hold off
    
    
    
    
=======
 
    figure;
    hold on
    for i=round(linspace(1,ind(end),10))
        R_cur=reshape(x(i,7:15),3,3);
        t_cur=x(i,1:3)';
        T_cur=[[R_cur t_cur]; 0 0 0 1];
        trplot(T_cur)
        
    end
>>>>>>> 9922288ccf8eb667e958caa9fb4b9a3e1a4abe4e
    
% % ANIMATION
% % =========
% keyboard;
% animate_3dquad(t(ind), x(ind,:),t(ind), xd(ind,:));


end

<<<<<<< HEAD
%%
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

% fetching desired states
% -----------------------
traj=get_flats(t);
xQd=traj.x;
vQd=traj.dx;
aQd=traj.d2x;
psid=traj.psi;
Omegad =zeros(3,1);



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

    k1 =  0.12*diag([4, 4 ,9.8055*1.2]);
    k2 = 0.5*diag([4, 4, 10]);
    A = (-k1*eQ - k2*deQ + (mQ)*(aQd + g*e3));
    normA = norm(A);
    b3d = A/norm(A);

    f = vec_dot(A,b3);
    
     % Attitude Control - to follow b1d specified trajectory 
%     b1d = e1;
%     b1c = -vec_cross(b3c,vec_cross(b3c,b1d));
%     b1c = b1c/norm(vec_cross(b3c,b1d));
%     Rc = [b1c vec_cross(b3c,b1c) b3c];
%     Rd = Rc;

     % Attitude Control - to follow yaw specified trajectory 
     b1c=[cos(psid) sin(psid) 0]';
     b2d=cross(b3d,b1c); b2d=b2d/norm(b2d);
     b1d=cross(b2d,b3d);
     
     Rd=[b1d b2d b3d];
    
    
    if(norm(Rd'*Rd-eye(3)) > 1e-2)
        disp('Error in R') ; keyboard ;
    end

    kR = 4 ; kOm = 4 ;
    epsilon = 0.1 ; %.5 ; %0.01 ;

    err_R = 1/2 * vee_map(Rd'*R - R'*Rd) ;
    err_Om = Omega - R'*Rd*Omegad ;
    M = -kR*err_R - kOm*err_Om  ;
    
    % Equations of Motion
    % -------------------
    xQ_dot = vQ;
    vQ_dot = -g*e3 + (f/mQ)*R*e3;
    
    
    R_dot = R*hat_map(Omega) ;
    Omega_dot = J\( -vec_cross(Omega, J*Omega) + M ) ;
    
    
% Computing xd
% ------------
xd = [xQd; vQd];
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



%%

function [traj] = get_flats(t)
traj.x = [6*t^2-4*t^3;6*t^2-4*t^3;6*t^2-4*t^3];
traj.dx = [12*t-12*t^2;12*t-12*t^2;12*t-12*t^2];
traj.d2x = [12-24*t;12-24*t;12-24*t];
traj.d3x = [-24;-24;-24];
traj.d4x = [0;0;0];

traj.psi = pi/80*t;
traj.dpsi = 60*t-60*t^2;
traj.d2psi = 60-120*t;
traj.d3psi = -120;
traj.d4psi = 0;
end

function traj_plot(tspan)
% tspan=[to tf]
figure
x=[]; y=[]; z=[];
for t=tspan(1):0.5:tspan(2)
    traj=get_flats(t);
    x=[x traj.x(1)];
    y=[y traj.x(2)];
    z=[z traj.x(3)];
end
plot3(x,y,z)

end







=======

>>>>>>> 9922288ccf8eb667e958caa9fb4b9a3e1a4abe4e
