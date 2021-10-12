%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                      Update Target System State                     %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
% * Description:                                                          %
%   - Updates the state of the target system using the uncertain model.   %
%                                                                         %
% * Inputs:                                                               %
%   - x_curr:    current state, x(k)                                      %
%   - u_curr:    current input, u(k)                                      %
%   - sys_model: system model structure                                   %
%   - Delta1:    contraction matrix (optional)                            %
%                                                                         %
% * Outputs:                                                              %
%   - x_new:    new state, x(k+1)                                         %
%   - w_k:      noise signal, w(k)                                        %     
%   - Delta1_k: contraction matrix                                        %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [x_new,w_k,Delta1_k] = ...
                    update_target_system(x_curr,u_curr,sys_model,Delta1)

% ======================================================================= %
% Generate uncertain parameter matrices
% ======================================================================= %

% Generate contraction (norm <= 1), if not given as argument
if (nargin < 4)
    Delta1_k = unifrnd(-1,1);
else
    Delta1_k = Delta1;
end

% Unpack system model matrices

F = sys_model.F;
G = sys_model.G;
H = sys_model.H;
Q = sys_model.Q;

M1 = sys_model.M1;
EF = sys_model.EF;
EG = sys_model.EG;
EH = sys_model.EH;

% Parameter uncertainty matrices

delta_F = M1 * Delta1_k * EF;
delta_G = M1 * Delta1_k * EG;
delta_H = M1 * Delta1_k * EH;

% Uncertain parameter matrices

F_r = F + delta_F;
G_r = G + delta_G;
H_r = H + delta_H;

% ======================================================================= %
% Generate noise signal
% ======================================================================= %

w_k = mvnrnd(zeros(size(Q,1),1), Q)';

% ======================================================================= %
% Simulate the uncertain target system
% ======================================================================= %

% Update the state
x_new = F_r * x_curr + G_r * u_curr + H_r * w_k;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end
