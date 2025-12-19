function [x_hat_k, P_k, S_k, y_hat_k_m] = EKF(x_hat_0_p, P_0_p, u, y_k, Q, R, const)
%% Description
% Runs Extended Kalman Filter on data
% INPUTS:
%   x_hat_0_p : State estimate at k = 0 (n x n)
%   P_0_p : Error covariance at k = 0 (n x n)
%   u : Input (Constant) (m x 1)
%   y_k : Output history k = 1,2,3,... (p x num_step)
%   Q : Process noise covariance matrix (Constant) (n x n)
%   R : Measurement noise covariance matrix (Constant) (p x p)
%   const : constants 
% OUTPUTS:
%   x_hat_k : State estimate history k = 0,1,2,3,... (n x num_step)
%   P_k : Error covariance history k = 0,1,2,3,... (n x n x num_step)

%% Variable definitions
n = const.n; % Number of states
p = const.p; % Number of outputs
num_step = const.num_step; % Number of time steps

x_hat_k = NaN(n,num_step);
P_k = NaN(n,n,num_step);
S_k = NaN(p,p,num_step);
y_hat_k_m = NaN(p,num_step);

%% First step
% Set initial values
x_hat_k(:,1) = x_hat_0_p;
P_k(:,:,1) = P_0_p;

x_hat_km1_p = x_hat_k(:,1);
P_km1_p = P_k(:,:,1);

%% Future steps
for lv1 = 2:num_step
    %% Compute partial derivative matrices
    F_km1 = calc_F_k(x_hat_km1_p, u, const);
    L_km1 = calc_L_k(n, const);
    
    %% Perform prediction update
    P_k_m = F_km1*P_km1_p*F_km1' + L_km1*Q*L_km1';
    x_hat_k_m = f_km1(x_hat_km1_p, u, zeros(n,1), const);
    
    %% Compute partial derivative matrices
    H_k = calc_H_k(x_hat_k_m);
    
    %% Perform measurement update 
    K_k = P_k_m*H_k'*(H_k*P_k_m*H_k' + R)^-1;

    % Compute difference from expected outputs
    y_delta = y_k(:,lv1) - h_k(x_hat_k_m, zeros(p,1));

    % Wrap output delta
    y_delta(1) = wrapToPi(y_delta(1));
    y_delta(3) = wrapToPi(y_delta(3));

    % Compute correction step
    x_hat_k_p = x_hat_k_m + K_k*(y_delta);

    % Wrap corrected state
    x_hat_k_p(3) = wrapToPi(x_hat_k_p(3));
    x_hat_k_p(6) = wrapToPi(x_hat_k_p(6));

    % Compute updated error covariance
    P_k_p = (eye(n) - K_k*H_k)*P_k_m;

    %% Save and advance step
    x_hat_k(:,lv1) = x_hat_k_p;

    P_k(:,:,lv1) = P_k_p;
    S_k(:,:,lv1) = H_k*P_k_m*H_k' + R; % Compute residuals
    y_hat_k_m(:,lv1) = h_k(x_hat_k_m, zeros(p,1));

    x_hat_km1_p = x_hat_k_p;
    P_km1_p = P_k_p;
end