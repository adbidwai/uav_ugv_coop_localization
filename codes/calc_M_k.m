function M_k = calc_M_k(p)
%% Description
% Computes linearized variance caused by process noise
% INPUTS
%   p : Number of outputs
% OUTPUTS
%   L_k : Matrix transformation on process noise to error covariance

%% Compute
M_k = eye(p,p);