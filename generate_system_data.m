%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                         Generate System Data                        %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
% * Description:                                                          %
%   - Generates the actual state evolution of the target system and       % 
%     all sensor measurements.                                            %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                                Setup                                %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ======================================================================= %
% Clear workspace
% ======================================================================= %

clc
clear
close all

% ======================================================================= %
% Simulation parameters
% ======================================================================= %

fprintf("--> Starting...\n");

example = 1;

N = 1000;
T = 1000;

% ======================================================================= %
% Get example initial parameters
% ======================================================================= %

fprintf("--> Defining simulation parameters for Example %d...\n", example);

k = 1;  % In case the system is time-varying, consider the initial step

addpath example_parameters;

filename = sprintf('example_parameters_%d', example);
eval(filename);

% ======================================================================= %
% Problem dimensions
% ======================================================================= %

n = size(F,1);  % System state
m = size(G,2);  % System input
p = size(H,2);  % System noise
r = size(C,1);  % Sensor measurements
q = size(D,2);  % Measurement noise

s1 = size(M1,2);
t1 = size(EF,1);
s2 = size(M2,2);
t2 = size(EC,1);

% ======================================================================= %
% Pre-allocation
% ======================================================================= %

% System state (all experiments)
x = zeros(n,N+2,T);

% System input (all experiments)
u = zeros(m,N+1,T);

% System noise (all experiments)
w = zeros(p,N+1,T);

% Sensor measurements (all experiments)
y = zeros(r,N+1,T);

% Measurement noise (all experiments)
v = zeros(q,N+1,T);

% Contraction matrices (all experiments)
Delta1 = zeros(s1,t1,N+1,T);
Delta2 = zeros(s2,t2,N+1,T);

% ======================================================================= %
% Data generation
% ======================================================================= %

fprintf("--> Generating system data...\n");

% Experiments loop
for e = 1:T

    fprintf('===== Experiment %d =====\n', e);

    % Initialize system state for current experiment
    x(:,1,e) = x0;

    % Time loop
    for k = 1:N+1
        % Get system parameters
        eval(filename);

        % Current system state
        x_k = x(:,k,e);

        % Update system input vector
        u(:,k,e) = u_k;

        % Update target system state
        [x(:,k+1,e), w(:,k,e), Delta1(:,:,k,e)] = ...
                                update_target_system(x_k,u_k,sys_model);

        % Obtain a measurement of the target system
        [y(:,k,e), v(:,k,e), Delta2(:,:,k,e)] = ...
                                        measure_target(x_k,sys_model);
    end
end

% ======================================================================= %
% Save data
% ======================================================================= %

filename = sprintf("saved_system_data/system_data_%d", example);

save(filename, 'x','u','w','y','v');
% save(filename, 'x','u','w','y','v','Delta1','Delta2');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
