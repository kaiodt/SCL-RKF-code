%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                        Measure Target System                        %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
% * Description:                                                          %
%   - Generate a sensor measurement of the target system.                 %
%                                                                         %
% * Inputs:                                                               %
%   - x_curr:    current system state, x(k)                               %
%   - sys_model: system model structure                                   %
%   - Delta2:    contraction matrix (optional)                            %
%                                                                         %
% * Outputs:                                                              %
%   - y_k:      measurement, y(k)                                         %
%   - v_k:      measurement noise signal, v(k)                            %
%   - Delta2_k: contraction matrix                                        %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [y_k,v_k,Delta2_k] = measure_target(x_curr,sys_model,Delta2)

% ======================================================================= %
% Generate uncertain parameter matrices
% ======================================================================= %

% Generate contraction (norm <= 1), if not given as argument
if (nargin < 3)
    Delta2_k = unifrnd(-1,1);
else
    Delta2_k = Delta2;
end

% Unpack sensing model matrices

C = sys_model.C;
D = sys_model.D;
R = sys_model.R;

M2 = sys_model.M2;
EC = sys_model.EC;
ED = sys_model.ED;

% Parameter uncertainty matrices

delta_C = M2 * Delta2_k * EC;
delta_D = M2 * Delta2_k * ED;

% Uncertain parameter matrices

C_r = C + delta_C;
D_r = D + delta_D;

% ======================================================================= %
% Generate noise signal
% ======================================================================= %

v_k = mvnrnd(zeros(size(R,1),1), R)';

% ======================================================================= %
% Generate measurement of the target system
% ======================================================================= %

y_k = C_r * x_curr + D_r * v_k;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end
