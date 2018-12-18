function output = controller_pd(input)

% parameters
x_dot_ref = input(1);
y_dot_ref = input(2);
z_dot_ref = input(3);
psi_dot_ref = input(4);

x_ref = input(5);
y_ref = input(6);
z_ref = input(7);
psi_ref = input(8);

x = input(9);
y = input(10);
z = input(11);
phi = input(12);
theta = input(13);
psi = input(14);

x_dot = input(15);
y_dot = input(16);
z_dot = input(17);
phi_dot = input(18);
theta_dot = input(19);
psi_dot = input(20);

x_dot_diff = x_dot_ref-x_dot;
x_diff = x_ref-x;

y_dot_diff = y_dot_ref-y_dot;
y_diff = y_ref-y;

if(abs(x_dot_diff) > 1)
   x_dot_diff = 1 * abs(x_dot_diff)/x_dot_diff; 
end

if(abs(x_diff) > 2)
   x_diff = 2 * abs(x_diff)/x_diff; 
end

if(abs(y_dot_diff) > 1)
   y_dot_diff = 1 * abs(y_dot_diff)/y_dot_diff; 
end

if(abs(y_diff) > 2)
   y_diff = 2 * abs(y_diff)/y_diff; 
end

a = 0.8*1e-1; %0.2 d게인?
b = 0.4*1e-1; %0.1 p게인?
phi_ref = a*(y_dot_diff)+b*(y_diff);
theta_ref = a*(x_dot_diff)+b*(x_diff);
% if(theta_ref > 10)
%    b = 1; 
% end

% if abs(theta_ref)>0.2
%         theta_ref = 0.2 * theta_ref/abs(theta_ref);
% end
convert = [cos(psi) sin(psi);sin(psi) -cos(psi)]*[theta_ref;phi_ref]; % WHAT 
theta_ref = convert(1);
phi_ref = convert(2);

% if abs(theta_ref)+abs(phi_ref) > 0.6
%     theta_ref = theta_ref / (abs(theta_ref)+abs(phi_ref)) * 0.6;
%     phi_ref = phi_ref / (abs(theta_ref)+abs(phi_ref)) * 0.6;
% end


% % Gain 7_14
% kp_phi = 30;    kd_phi = 10;
% kp_theta = 30;  kd_theta = 10;
% kp_psi = 3;    kd_psi = 2.5;
% kp_z = 5;      kd_z = 3;    KI_z = 3; 

% Gain
kp_phi = 30;    kd_phi = 20;
kp_theta = 30;  kd_theta = 20;
kp_psi = 3;    kd_psi = 2.5;
kp_z = 5;      kd_z = 3;    KI_z = 3; 



% global I_ze;
I_ze = 0;
ze = z_ref-z; % ze 는 z_error
% I_ze = I_ze+ze; 


% input
u1 = kp_z*(ze)+kd_z*(z_dot_ref-z_dot)+KI_z*(I_ze)+9.81;
u2 = kp_phi*(phi_ref-phi)+kd_phi*(-phi_dot);
u3 = kp_theta*(theta_ref-theta)+kd_theta*(-theta_dot);
u4 = kp_psi*(psi_ref-psi)+kd_psi*(psi_dot_ref-psi_dot);
% u2 = 0;
% u3 = 0;
% u4 = 0;
if u1>20
    u1 = 20;
elseif u1<0
    u1 = 0;
end

% if abs(u2)>2
%         u2 = 2 * u2/abs(u2);
% end
% 
% if abs(u3)>2
%         u3 = 2 * u3/abs(u3);
% end
% 
% if abs(u4)>2
%         u4 = 2 * u4/abs(u4);
% end

output = [u1,u2,u3,u4];