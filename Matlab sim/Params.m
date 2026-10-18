% Bobrow et al., Table I, PDF p. 2. SI units throughout.
g = 9.81;          % m/s^2

%geometry
l = 0.15;         % cube side, m
d = l*sqrt(2)/2;   % pivot-to-center distance for both bodies, m

%masses
mw = 0.15;        % wheel mass, kg
mb = 0.70;        % body mass (excluding wheel), kg
mtot = mw + mb;

%inertias
Ibg = 3.75e-3;     % body inertia at center of mass, kg*m^2
Iwg = 1.25e-4;     % wheel axial inertia at center of mass, kg*m^2
Ibo = Ibg + mb*d^2;
Iwo = Iwg + mw*d^2;
Itot = Ibo + Iwo;
IcO_bar = Itot - Iwg; % pivot inertia excluding wheel axial spin

% Motor friction, paper Eq. (36) and Table II (experimentally identified):
% tau_f(w) = sign(w)*(tau_c + bw*|w| + cd*w^2), w = thetawdot (relative).
tau_c = 2.46e-3;   % Coulomb friction, N*m
bw = 1.06e-5;      % viscous friction, N*m*s/rad
cd = 1.70e-8;      % aerodynamic drag, N*m*s^2/rad^2

% sign() is smoothed as tanh(w/w_eps) to avoid solver chattering at w = 0.
w_eps = 0.5;       % rad/s
% Pivot (edge) damping on the body. Not in the paper: 0 keeps it paper-exact.
bc = 0;            % N*m*s/rad



tau_input = 0; % motor torque, N*m


% Paper convention: thetab = theta_c is the angle from the horizontal
% world axis to a cube face (Fig. 4). Upright balance is thetab = pi/4.
% Positive tau drives the wheel and applies negative reaction torque to body.
gravity_gain = mtot*g*d;
% Center-of-mass angle = thetab + com_angle_offset (used by Simulink Bias,
% the ODE, and the animation). 0 matches the printed Eq. (7) cos(theta_c),
% where upright is thetab = pi/2; pi/4 matches Eq. (6), upright at pi/4.
com_angle_offset = 0;
body_accel_gain = 1/IcO_bar;
wheel_accel_gain = 1/Iwg;
M = [IcO_bar Iwg; 0 Iwg]; % PDF Eq. (9), inertia coefficients in kg*m^2


% Initial conditions
% Edit these four values to set the body and wheel initial state.
% A value of 40 deg starts 5 deg away from the unstable 45 deg equilibrium.
thetab0 = deg2rad(45); % paper theta_c, rad
thetabdot0 = 0;        % rad/s

thetaw0 = 0;           % wheel angle relative to body, rad
thetawdot0 = 0;        % wheel speed relative to body, rad/s

% Optional state vector if using MATLAB function model
x0 = [thetab0;
      thetabdot0;
      thetaw0;
      thetawdot0];

% Exact matrix form if needed later.
% This matrix maps:
% [thetabddot;
%  thetabddot + thetawddot]
M_paper = [IcO_bar, Iwg;
           0,       Iwg];