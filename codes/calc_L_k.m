function L_k = calc_L_k(n,const)
%% Description
% Computes linearized variance caused by process noise
% INPUTS
%   n : Number of states
%   const : Constant parameters
% OUTPUTS
%   L_k : Matrix transformation on process noise to error covariance

%% Compute
L_k = eye(n,n).*const.Dt;