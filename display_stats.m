%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                    Display Simulation Statistics                    %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
% * Description:                                                          %
%   - Displays a series of estimation statistics for all filters.         %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ======================================================================= %
% Compute simulation statistics
% ======================================================================= %

% Pre-allocating lists
f_ids = cell(n_filters,1);      % Filter identifications
f_means = zeros(n_filters,1);   % Mean of error variances
f_stds = zeros(n_filters,1);    % Standard deviation of error variances
f_times = zeros(n_filters,1);   % Average iteration execution times

for f = 1:n_filters
    f_ids{f} = filters{f}.id;
    f_means(f) = filters{f}.mean_S2;
    f_stds(f) = filters{f}.std_S2;
    f_times(f) = filters{f}.T_it * 1e3;
end

% ======================================================================= %
% Display results
% ======================================================================= %

fprintf("########## Simulation Statistics ##########\n\n");

VarNames = {'Filter','Mean_dB','Std_Dev_dB','T_iter_ms'};

sim_stats = table(f_ids,f_means,f_stds,f_times,'VariableNames',VarNames);

disp(sim_stats);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
