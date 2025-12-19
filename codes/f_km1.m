function x_k = f_km1(x_km1, u_km1, w_km1, const)
%% Description
% Full nonlinear dynamics function
%
% INPUTS:
%   x_km1 : State at time step k-1
%   u_km1 : Inputs at time step k-1
%   w_km1 : Process noise at time step k-1
%   const : Constant parameters
% OUTPUTS
%   x_k : State at time step k
%
% Michael States
% Updated Dec 2025

%% Setup integration
Dt = const.Dt; % Time step
options = odeset('AbsTol',3e-14,'RelTol',3e-14); % Set integration tolerence

[~, x_k] = ode45(@ODEs_f_km1, [0 Dt/2 Dt], x_km1, options, u_km1, w_km1, const);

x_k = x_k(end,:)'; % Take transpose and final value (ODE45 outputs row vector)

% Wrap
x_k(3,:) = wrapToPi(x_k(3,:));
x_k(6,:) = wrapToPi(x_k(6,:));