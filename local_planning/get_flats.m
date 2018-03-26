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