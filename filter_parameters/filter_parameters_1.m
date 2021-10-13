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
% Common parameters
% ======================================================================= %

% Initial state estimate
xp0 = zeros(n,1);

% Initial estimation error weighting matrix
P0 = 1 * eye(n);

% ======================================================================= %
% Kalman Filter (KF)
% ======================================================================= %

% ----------------------------------------------------------------------- %
% Initialization parameters
% ----------------------------------------------------------------------- %

KF_init_params = struct();

% Initial state estimate
KF_init_params.xp0 = xp0;

% Initial estimation error weighting matrix
KF_init_params.P0 = P0;

% ----------------------------------------------------------------------- %
% Filter parameters
% ----------------------------------------------------------------------- %

KF_params = struct();

% ======================================================================= %
% Robust Kalman Filter (RKF)
% ======================================================================= %

% ----------------------------------------------------------------------- %
% Initialization parameters
% ----------------------------------------------------------------------- %

RKF_init_params = struct();

% Initial state estimate
RKF_init_params.xp0 = xp0;

% Initial estimation error weighting matrix
RKF_init_params.P0 = P0;

% ----------------------------------------------------------------------- %
% Filter parameters
% ----------------------------------------------------------------------- %

RKF_params = struct();

% Penalty parameter
RKF_params.mu = 1;

% ksi parameter (lambda approximation)
RKF_params.ksi = 0.1;

% ======================================================================= %
% Optimal Robust Kalman Filter (ORKF)
% ======================================================================= %

% ----------------------------------------------------------------------- %
% Initialization parameters
% ----------------------------------------------------------------------- %

ORKF_init_params = struct();

% Initial state estimate
ORKF_init_params.xp0 = xp0;

% Initial estimation error weighting matrix
ORKF_init_params.P0 = P0;

% ----------------------------------------------------------------------- %
% Filter parameters
% ----------------------------------------------------------------------- %

ORKF_params = struct();

% Penalty parameter
ORKF_params.mu = 1e16;

% ksi parameter (lambda approximation)
ORKF_params.ksi = 0.1;

% ======================================================================= %
% Bounded Data Uncertainty Filter (BDU)
% ======================================================================= %

% ----------------------------------------------------------------------- %
% Initialization parameters
% ----------------------------------------------------------------------- %

BDU_init_params = struct();

% Initial state estimate
BDU_init_params.xp0 = xp0;

% Initial estimation error weighting matrix
BDU_init_params.P0 = P0;

% ----------------------------------------------------------------------- %
% Filter parameters
% ----------------------------------------------------------------------- %

BDU_params = struct();

% ksi parameter (lambda approximation)
BDU_params.ksi = 0.5;

% ======================================================================= %
% Guaranteed Cost Filter (GCF)
% ======================================================================= %

% ----------------------------------------------------------------------- %
% Initialization parameters
% ----------------------------------------------------------------------- %

GCF_init_params = struct();

% Initial state estimate
GCF_init_params.xp0 = xp0;

% Initial estimation error weighting matrix
GCF_init_params.P0 = P0;

% ----------------------------------------------------------------------- %
% Filter parameters
% ----------------------------------------------------------------------- %

GCF_params = struct();

% epsilon parameters (approximations)
GCF_params.eps_a = 0.01;
GCF_params.eps_b = 0.01;

% ======================================================================= %
% LMI-Based Robust Kalman Filter (LMI_RKF)
% ======================================================================= %

% ----------------------------------------------------------------------- %
% Initialization parameters
% ----------------------------------------------------------------------- %

LMI_RKF_init_params = struct();

% Initial state estimate
LMI_RKF_init_params.xp0 = xp0;

% Initial estimation error weighting matrix
LMI_RKF_init_params.P0 = P0;

% ----------------------------------------------------------------------- %
% Filter parameters
% ----------------------------------------------------------------------- %

LMI_RKF_params = struct();

% Penalty parameter
LMI_RKF_params.mu = 0.1;

% ni parameter (approximations)
LMI_RKF_params.ni = 1;

% ======================================================================= %
% Risk-Sensitive Kalman Filter (RSKF)
% ======================================================================= %

% ----------------------------------------------------------------------- %
% Initialization parameters
% ----------------------------------------------------------------------- %

RSKF_init_params = struct();

% Initial state estimate
RSKF_init_params.xp0 = xp0;

% Initial least-favorable conditioned variance
RSKF_init_params.V0 = 8 * eye(n);

% ----------------------------------------------------------------------- %
% Filter parameters
% ----------------------------------------------------------------------- %

RSKF_params = struct();

% tau parameter
RSKF_params.tau = 0.1;

% Tolerance parameter
RSKF_params.c = 2.9;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
