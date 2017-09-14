function[varargout] =  main_quad(varargin)
% Geometric control of Quadrotor on SE(3)
% http://www.math.ucsd.edu/~mleok/pdf/LeLeMc2010_quadrotor.pdf
% 
% Hybrid Robotics Lab
% Carnegie Mellon University
% Author: vkotaru@andrew.cmu.edu
% Date: June-8-2016
% Last Updated: June-8-2016

%% INITIALZING WORKSPACE
% ======================
% Clear workspace
% ---------------
% clear; 
close all; 
% clc;

% Add Paths
% ----------
% Adding path to 'Geometric Control Toolbox'
addpath('./Geometry-Toolbox/');


%trajectory_gen
%first run the code in main.m section new_trial
%% INITIALZING PARAMETERS
% ======================
% System constants and parameters
data.params.mQ = 0.5 ;
data.params.J = diag([0.557, 0.557, 1.05]*10e-2);
data.params.g = 9.81 ;
data.params.e1 = [1;0;0] ;
data.params.e2 = [0;1;0] ;
data.params.e3 = [0;0;1] ;
data.params.p=p;
data.params.n=n;

%% INTIALIZING - INTIAL CONDITIONS
% ================================
% Zero Position 
% -------------
xQ0 = [];
vQ0 = zeros(3,1);

R0 = RPYtoRot_ZXY(010*pi/180,0*pi/180, 0*pi/180) ;
Omega0 = zeros(3,1);

xQ0 = x0; 
vQ0 = zeros(3,1);
aQ0=zeros(3,1);
% 
R0 = eye(3) ;
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
x_init = [xQ0; vQ0 ;reshape(R0,9,1); Omega0 ];

%% SIMULATION
% ==========
disp('Simulating...') ;
odeopts = odeset('RelTol', 1e-9, 'AbsTol', 1e-10) ;
% odeopts = [] ;
tspan=[t0_real tf_real];
data.params.tspan=tspan;
[t, x] = ode15s(@odefun_quadDynamics, tspan, x_init, odeopts, data) ;
%%
% Computing Various Quantities
disp('Computing...') ;
ind = round(linspace(1, length(t),length(t))) ;
% ind = 0:length(t);
for i = ind
   [dx_,xd_,f_,M_] =  odefun_quadDynamics(t(i),x(i,:)',data);
   dx(i,:)=dx_';
   xd(i,:) = xd_';
   psi_exL(i) = norm(x(i,1:3)-xd(i,1:3));
   psi_evL(i) = norm(x(i,4:6)-xd(i,4:6));
   psi_eaL(i)=norm(x(i,7:9)-xd(i,7:9));
   f(i,1)= f_;
end


%%PLOTS
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
    plot3(x(ind,1),x(ind,2),x(ind,3),'-g',xd(ind,1),xd(ind,2),xd(ind,3),':r');
    hold on
%     for i=round(linspace(1,length(ind),10))
%         quiver(x(i,1),x(i,2),x(i,3),x(i,7),x(i,8),x(i,9),'r')
%     end
    grid on; title('trajectory');legend('traj','traj_d');%axis equal;
    
    xlabel('x-axis');ylabel('y-axis');zlabel('z-axis');
    
    
%%  
        figure;
    subplot(2,2,1);
    plot(t(ind),dx(ind,1),'-g',t(ind),xd(ind,4),':r');
    grid on; title('x');legend('x','x_d');%axis equal;
    xlabel('time');ylabel('x [m]');
    subplot(2,2,2);
    plot(t(ind),dx(ind,2),'-g',t(ind),xd(ind,5),':r');
    grid on; title('y');legend('y','y_d');%axis equal;
    xlabel('time');ylabel('y [m]');
    subplot(2,2,3);
    plot(t(ind),dx(ind,3),'-g',t(ind),xd(ind,6),':r');
    grid on; title('z');legend('z','z_d');%axis equal;
    xlabel('time');ylabel('z [m]');
    subplot(2,2,4);
    plot3(x(ind,1),x(ind,2),x(ind,3),'-g',xd(ind,1),xd(ind,2),xd(ind,3),':r');
    hold on
%     for i=round(linspace(1,length(ind),10))
%         quiver(x(i,1),x(i,2),x(i,3),x(i,7),x(i,8),x(i,9),'r')
%     end
    grid on; title('trajectory');legend('traj','traj_d');%axis equal;
    
    xlabel('x-axis');ylabel('y-axis');zlabel('z-axis');
    
    
    
    
%% acceleration ratio
   ind = round(linspace(round(length(t)/1.2), length(t), 1000)) ;

    figure;
    subplot(2,1,1);
    
    plot(t(ind),dx(ind,5)./dx(ind,4),'-g',t(ind),zd(2)/zd(1)*ones(1,1000),':r');
    str = {'$$ \ddot{y}/\ddot{x} $$', '$$ \ddot{y}_{d}/\ddot{x}_{d}  $$'};
    grid on; title(str{1},'Interpreter','latex');
    legend(str, 'Interpreter','latex')

    xlabel('time');ylabel('d2x [m]');
    axis([t(ind(1))  t(ind(end)) 0 10 ])
    subplot(2,1,2);
    plot(t(ind),(dx(ind,6)+9.81)./dx(ind,4),'-g',t(ind),zd(3)/zd(1)*ones(1,1000),':r');
    str = {'$$ \ddot{z}/\ddot{x} $$', '$$ \ddot{z}_{d}/\ddot{x}_{d}  $$'};

    grid on; title(str{1},'Interpreter','latex');     legend(str, 'Interpreter','latex')%axis equal;
    xlabel('time');ylabel('y [m]');
    
    axis([t(ind(1))  t(ind(end)) 0 10 ])
    
%%
    
    
    
    
    
    
    figure;
    subplot(2,1,1);
    plot(t(ind),psi_exL(ind));
    grid on; title('position error');legend('psi-exL');
    subplot(2,1,2);
    plot(t(ind),psi_eaL(ind));
    grid on; title('acceleration error');legend('psi-eaL');
 
% % ANIMATION
% % =========
% keyboard;
% animate_3dquad(t(ind), x(ind,:),t(ind), xd(ind,:));


end

%%
function[traj] = get_flats(t)

traj.x = [6*t^2-4*t^3;6*t^2-4*t^3;6*t^2-4*t^3];
traj.dx = [12*t-12*t^2;12*t-12*t^2;12*t-12*t^2];
traj.d2x = [12-24*t;12-24*t;12-24*t];
traj.d3x = [-24;-24;-24];
traj.d4x = [0;0;0];

traj.psi = 30*t^2-20*t^3;
traj.dpsi = 60*t-60*t^2;
traj.d2psi = 60-120*t;
traj.d3psi = -120;
traj.d4psi = 0;

end
