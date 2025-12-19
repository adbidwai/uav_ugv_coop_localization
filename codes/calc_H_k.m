function H_k = calc_H_k(x_hat_k)
%% Description
% Solve for linearized observation function given estimate at time k
% INPUTS
%   x_hat_k : State estimate at time k
% OUTPUTS
%   H_k : Linearized estimation matrix at time k
%
% Michael States
% Updated Dec 2025

%% Evaluate
% Grab variables from state
xi_g = x_hat_k(1);
eta_g = x_hat_k(2);
xi_a = x_hat_k(4);
eta_a = x_hat_k(5);

% Calculate C matrix
Deta = eta_g - eta_a;
Dxi = xi_g - xi_a;
Dsqr = Deta^2 + Dxi^2; % Always positive

% Calculate H matrix
H_k = [-Deta/Dsqr, -1/(-Dxi + (-Deta)^2/-Dxi), -1, Deta/Dsqr, 1/(-Dxi + (-Deta)^2/-Dxi), 0;...
    Dxi/sqrt(Dsqr), Deta/sqrt(Dsqr), 0, -Dxi/sqrt(Dsqr), -Deta/sqrt(Dsqr), 0;...
    -Deta/Dsqr, 1/(Dxi + (Deta)^2/Dxi), 0, Deta/Dsqr, -1/(Dxi + (Deta)^2/Dxi), -1;...
    0 0 0 1 0 0;...
    0 0 0 0 1 0];