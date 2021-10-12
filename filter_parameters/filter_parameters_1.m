%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                    Filter Parameters - Example 1                    %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
% * Adapted from:                                                         %
%   L. Xie, Y.C. Soh, C. de Souza. Robust Kalman Filtering for Uncertain  %
%   Discrete-Time Systems. IEEE Trans. Automat. Control. 39 (6) (1994).   % 
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ======================================================================= %
% Kalman Filter (KF)
% ======================================================================= %

% Initial estimation error weighting matrix
P0_KF = 1 * eye(n);

% ======================================================================= %
% Robust Kalman Filter (RKF)
% ======================================================================= %

% Initial estimation error weighting matrix
P0_RKF = 1 * eye(n);

% Penalty parameter
mu_RKF = 1;

% ksi parameter (lambda approximation)
ksi_RKF = 0.1;

% ======================================================================= %
% Optimal Robust Kalman Filter (ORKF)
% ======================================================================= %

% Penalty parameter
mu_ORKF = 1e16;

% ksi parameter (lambda approximation)
ksi_ORKF = 0.1;

% ======================================================================= %
% Bounded Data Uncertainty Filter (BDU)
% ======================================================================= %

% ksi parameter (lambda approximation)
ksi_BDU = 0.5;

% ======================================================================= %
% Guaranteed Cost Filter (GCF)
% ======================================================================= %

% Epsilon parameters (approximations)
eps_a = 0.01;
eps_b = 0.01;

% ======================================================================= %
% LMI-Based Robust Kalman Filter (LMI-RKF)
% ======================================================================= %

% Penalty parameter
mu_LMI_RKF = 0.1;

% ni parameter (approximations)
ni_LMI_RKF = 1;

% ======================================================================= %
% Risk-Sensitive Kalman Filter (RSKF)
% ======================================================================= %

% Initial least-favorable conditioned variance
V0_RSKF = 8 * eye(2);

% tau parameter
tau_RSKF = 1;

% Tolerance parameter
c_RSKF = 2.9;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
