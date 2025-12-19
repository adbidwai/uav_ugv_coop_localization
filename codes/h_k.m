function y = h_k(x_k, v_k)
%% Description
% Full nonlinear dynamics function
%
% INPUTS:
%   x_k : State at time step k
%   v_k : Measurement noise at time step k
% OUTPUTS
%   y_k : Measurement at time step k
%
% Michael States
% Updated Dec 2025

%% Extract values from x_k and v_k
xi_g = x_k(1); % UGV East pos (m)
eta_g = x_k(2); % UGV North pos (m)
theta_g = x_k(3); % UGV direction (rad)
xi_a = x_k(4); % UAV East pos (m)
eta_a = x_k(5); % UAV North pos (m)
theta_a = x_k(6); % UAV direction (rad)

% y1 = atan((eta_a - eta_g)/(xi_a - xi_g)) - theta_g + v_k(1); % Heading towards UAV from UGV (rad)
y1 = wrapToPi(atan2((eta_a - eta_g),(xi_a - xi_g)) - theta_g + v_k(1));

y2 = sqrt((xi_g - xi_a)^2 + (eta_g - eta_a)^2) + v_k(2); % Range between UGV and UAV (m)
% y3 = atan((eta_g - eta_a)/(xi_g - xi_a)) - theta_a + v_k(3); % Heading towards UGV from UAV (rad)
y3 = wrapToPi(atan2((eta_g - eta_a),(xi_g - xi_a)) - theta_a + v_k(3)); 

y4 = xi_a + v_k(4); % UAV east pos (m)
y5 = eta_a + v_k(5); % UAV north pos (m)

y = [y1; y2; y3; y4; y5];