function F_k = calc_F_k(x_hat_k, u_k, const)
%% Description
% Solve for linearized dynamics function given estimate at time k
% INPUTS
%   x_hat_k : State estimate at time k
%   u_k : Inputs at time k
%   const : Constants struct
% OUTPUTS
%   F_k : Linearized dynamics at time k
%
% Michael States
% Updated Dec 2025

%% Evaluate
% Grab variables from state
theta_g = x_hat_k(3);
theta_a = x_hat_k(6);
Dt = const.Dt;
n = const.n;

% Grab variables from inputs
v_g = u_k(1);
v_a = u_k(3);

% Calculate F matrix
A_tilde_k = [0 0 -v_g*sin(theta_g) 0 0 0;...
    0 0 v_g*cos(theta_g) 0 0 0;...
    0 0 0 0 0 0;...
    0 0 0 0 0 -v_a*sin(theta_a);...
    0 0 0 0 0 v_a*cos(theta_a);...
    0 0 0 0 0 0];

% F_k = expm(A_tilde_k*Dt);
F_k = eye(n,n) + A_tilde_k*Dt;