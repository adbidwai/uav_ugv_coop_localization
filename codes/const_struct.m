% Constant parameters

%% UGV Parameters
const.L = 0.5; % UGV Wheelbase (m)
const.xi_g_nom_0 = 10; % UGV initial East position STATE (m)
const.eta_g_nom_0 = 0; % UGV initial North position STATE (m)
const.theta_g_nom_0 = pi/2; % UGV initial direction STATE (rad)
const.v_g = 2; % UGV velocity INPUT (m/s)
const.phi_g = -pi/18; % UGV turning angle INPUT (rad)

%% UAV parameters
const.xi_a_nom_0 = -60; % UAV initial East position STATE (m)
const.eta_a_nom_0 = 0; % UAV initial North position STATE (m)
const.theta_a_nom_0 = -pi/2; % UAV initial direction STATE (rad)
const.v_a = 12; % UAV velocity INPUT (m/s)
const.omega_a = pi/25; % UAV rotation rate INPUT (rad/s)

%% General Parameters
const.t0 = 0; % Initial Time (s)
const.tf = 200; % Final time (s)
const.Dt = .1; % Time step (s)
const.t = const.t0:const.Dt:const.tf; % Time steps (s)
const.num_step = length(const.t); % Number of steps (including 0)
const.n = 6; % Number of states
const.m = 4; % Number of inputs
const.p = 5; % Number of outputs
const.N = 50; % Number of Monte Carlo simulations
const.Qtune = diag([.08 .08 .006 .006 .006 .006]); % Process noise covariance 
const.Rtune = diag([.0225 64 .04 36 36]); % Measurement noise covariance

%% Monte Carlo Parameters
const.x_hat_0 = [10; 0; pi/2; -60; 0; -pi/2];
const.P_0_p = diag([1 1 .2 1 1 .2]);