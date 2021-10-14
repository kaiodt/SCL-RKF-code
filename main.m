%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                                Main                                 %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
% * Description:                                                          %
%   - Executes the simulations and shows the results                      %
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

% warning("off");
fprintf("--> Starting...\n\n");

% ======================================================================= %
% Set simulation parameters
% ======================================================================= %

fprintf("--> Setting simulation parameters...\n\n");

sim_parameters;

fprintf("* Example: %d\n", example);
fprintf("* Time Horizon: %d\n", N);
fprintf("* No. Experiments: %d\n", T);
fprintf("* Filters: ");

for f = 1:length(sim_filters)
    fprintf("%s ", sim_filters(f));
end

fprintf("\n\n");

% ======================================================================= %
% Load pre-generated system data
% ======================================================================= %

addpath saved_system_data;

fprintf("--> Loading system data...\n\n");

try
    filename = sprintf("system_data_%d", example);
    load(filename);
catch
    error("There is no data for example %d. Try a different one.",...
          example);
end

% Test if N and T are valid values

if N > size(x,2)-2
    error("Maximum available time horizon is N = %d.", size(x,2)-2);
end

if T > size(x,3)
    error("Maximum available number of experiments is T = %d.", size(x,3));
end

% ----------------------------------------------------------------------- %
% Problem dimensions
% ----------------------------------------------------------------------- %

n = size(x,1);  % System state

% ======================================================================= %
% Select example parameters file
% ======================================================================= %

addpath example_parameters;

% Get system parameters
try
    example_file = sprintf("example_parameters_%d", example);
    eval(example_file);
catch
    error("There are no parameters defined for example %d.", example);
end

% ======================================================================= %
% Set filter parameters
% ======================================================================= %

addpath filter_parameters;

fprintf("--> Setting filter parameters...\n\n");

try
    filename = sprintf("filter_parameters_%d", example);
    eval(filename);
catch
    error("There are no filter parameters defined for example %d.",...
          example);
end    

% ======================================================================= %
% Instantiate filters
% ======================================================================= %

addpath filters;

% Number of filters
n_filters = length(sim_filters);

% Cell containing each filter instance
filters = cell(1,n_filters);

% Cell containing the initialization parameters of each filter
f_init_params = cell(1,n_filters);

% Cell containing the parameters of each filter
f_params = cell(1,n_filters);

% Populate cells according to selected filters
for f = 1:n_filters
    try
        f_inst_str = sprintf("%s(N,T,n);", sim_filters(f));
        filters{f} = eval(f_inst_str);

        f_init_str = sprintf("%s_init_params", sim_filters(f));
        f_init_params{f} = eval(f_init_str);

        f_params_str = sprintf("%s_params", sim_filters(f));
        f_params{f} = eval(f_params_str);

        % Pre-compute LMIs if necessary
        if strcmp(sim_filters(f), "LMI_RKF")
            filters{f}.pre_compute_lmis(N,sys_model,...
                                        f_init_params{f},f_params{f});
        end
    catch
        error("Filter %s is not implemented!", sim_filters(f));
    end
end

fprintf("--> Filters ready!\n\n");

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                           Simulation Loop                           %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fprintf("--> Starting simulation...\n\n");

% ======================================================================= %
% Start timer
% ======================================================================= %

tic

% ======================================================================= %
% Experiments loop
% ======================================================================= %

for e = 1:T

    fprintf("========== Experiment %d ==========\n", e);

    % Initialize filters

    for f = 1:n_filters
        filters{f}.initialize(f_init_params{f});
    end

    % =================================================================== %
    % Time loop
    % =================================================================== %

    for k = 1:N+1

        % Get system parameters
        eval(example_file);

        % Current input
        u_k = u(:,k,e);

        % Current measurement
        y_k = y(:,k,e);

        % Update filter estimates

        for f = 1:n_filters
            filters{f}.update_estimate(e,k,u_k,y_k,sys_model,f_params{f});
        end

    end % Time loop

    % ------------------------------------------------------------------- %
    % Update estimation errors
    % ------------------------------------------------------------------- %

    % System state sequence for current experiment
    x_e = x(:,1:N+1,e);

    % Update estimation errors of each filter
    for f = 1:n_filters
        filters{f}.update_error_variance(e,x_e);
    end

end % Experiments loop

fprintf("\n");
fprintf("--> Simulation finished!\n\n");

% ======================================================================= %
% Stop timer
% ======================================================================= %

toc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                               Results                               %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ======================================================================= %
% Compute simulation statistics
% ======================================================================= %

fprintf("\n");
fprintf("--> Computing statistics...\n\n");

% Compute final statistics of each filter
for f = 1:n_filters
    filters{f}.compute_filter_stats(T,N);
end

% Display statistics
display_stats;

% ======================================================================= %
% Plot results
% ======================================================================= %

fprintf("--> Plotting results...\n");

plot_results;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
