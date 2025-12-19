function [A, B, C] = eval_jacobian(x, u, L)
% eval_jacobian  
% [A, B] = eval_jacobian(x, u)
% Outputs A(t) and B(t) for the UGV UAV problem given the current time step state and inputs. 
%
% INPUT PARAMETERS:
% x = current state (at t)
% u = current input (at t)
% L = wheelbase (m)
%
% OUTPUT PARAMETERS:
% A = A(t)
% B = B(t)
%
% Michael States
% Updated December 2025
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Grab variables from state
xi_g = x(1);
eta_g = x(2);
theta_g = x(3);
xi_a = x(4);
eta_a = x(5);
theta_a = x(6);

% Grab variables from inputs
v_g = u(1);
phi_g = u(2);
v_a = u(3);
omega_a = u(4);

% Calculate A matrix
A = [0 0 -v_g*sin(theta_g) 0 0 0;...
    0 0 v_g*cos(theta_g) 0 0 0;...
    0 0 0 0 0 0;...
    0 0 0 0 0 -v_a*sin(theta_a);...
    0 0 0 0 0 v_a*cos(theta_a);...
    0 0 0 0 0 0];

% Calculate B matrix
B = [cos(theta_g) 0 0 0;...
    sin(theta_g) 0 0 0;...
    1/L*tan(phi_g) v_g/L*sec(phi_g)^2 0 0;...
    0 0 cos(theta_a) 0;...
    0 0 sin(theta_a) 0;...
    0 0 0 1];

% Calculate C matrix
Deta = eta_g - eta_a;
Dxi = xi_g - xi_a;
Dsqr = Deta^2 + Dxi^2; % Always positive

C = [-Deta/Dsqr, -1/(-Dxi + (-Deta)^2/-Dxi), -1, Deta/Dsqr, 1/(-Dxi + (-Deta)^2/-Dxi), 0;...
    Dxi/sqrt(Dsqr), Deta/sqrt(Dsqr), 0, -Dxi/sqrt(Dsqr), -Deta/sqrt(Dsqr), 0;...
    -Deta/Dsqr, 1/(Dxi + (Deta)^2/Dxi), 0, Deta/Dsqr, -1/(Dxi + (Deta)^2/Dxi), -1;...
    0 0 0 1 0 0;...
    0 0 0 0 1 0];