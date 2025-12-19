function x_dot = ODEs_f_km1(t, x, u, w, const)
%% Description
% Nonlinear dynamics differential equation setup for ODE45
% INPUTS:
%   x : state
%   u : input (ZOH)
%   w : process noise (ZOH)
% OUTPUTS
%   x_dot : time derivative of state
%
% Michael States
% Updated Dec 2025

%% Definitions
% Extract values from x, u, w, and const
xi_g = x(1); % UGV East pos (m)
eta_g = x(2); % UGV North pos (m)
theta_g = x(3); % UGV direction (rad)
xi_a = x(4); % UAV East pos (m)
eta_a = x(5); % UAV North pos (m)
theta_a = x(6); % UAV direction (rad)

v_g = u(1); % UGV vel (m/s)
phi_g = u(2); % UGV steering angle (rad)
v_a = u(3); % UAV vel (m/s)
omega_a = u(4); % UAV turning rate (rad/s)

w_xi_g = w(1); % UGV East pos process noise (m)
w_eta_g = w(2); % UGV North pos process noise (m)
w_theta_g = w(3); % UGV direction process noise (rad)
w_xi_a = w(4); % UAV East pos process noise (m)
w_eta_a = w(5); % UAV North pos process noise (m)
w_theta_a = w(6); % UAV turning rate process noise (rad/s)

L = const.L; % UGV wheelbase (m)

%% Computations
% Define derivatives (from project description)
% UGV
xi_g_dot = v_g*cos(theta_g) + w_xi_g;
eta_g_dot = v_g*sin(theta_g) + w_eta_g;
theta_g_dot = v_g/L*tan(phi_g) + w_theta_g;

xi_a_dot = v_a*cos(theta_a) + w_xi_a;
eta_a_dot = v_a*sin(theta_a) + w_eta_a;
theta_a_dot = omega_a + w_theta_a;

%% Output
% Combine into state vector
x_dot = [xi_g_dot; eta_g_dot; theta_g_dot; xi_a_dot; eta_a_dot; theta_a_dot];