%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                        Simulation Parameters                        %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
% * Description:                                                          %
%   - Set desired simulation parameters.                                  %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Simulation example
example = 1;

% Time horizon
N = 100;

% Number of experiments
T = 10;

% Choose which filters to simulate
% Available filters:
% KF      - Kalman Filter
% RKF     - Robust Kalman Filter
% ORF     - Optimal Robust Filter (Ishihara, 2015)
% BDU     - Bounded Data Uncertainty Filter (Sayed, 2001)
% GCF     - Guaranteed Cost Filter (Dong, 2006)
% LMI_RKF - LMI-Based Robust Kalman Filter (Abolhasani, 2018)

sim_filters = ["KF","RKF","ORF","BDU","GCF","LMI_RKF"];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
