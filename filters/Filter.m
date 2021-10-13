%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                          Filter Base Class                          %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
% * Description:                                                          %
%   - Base class with properties and methods common to most filters.      %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef Filter < handle

    % =================================================================== %

    properties
        id; % Filter identification

        xp; % Predicted estimation
        xf; % Filtered estimation

        S1; % Estimation error variance of each variable (ensemble average)
        S2; % Estimation error variance of whole state (ensemble average)

        mean_S2; % Mean of estimation error variance
        std_S2;  % Standard deviation of estimation error variance

        T_ex;    % Total execution time (one experiment)
        T_it;    % Average iteration execution time
    end

    % =================================================================== %

    % =================================================================== %

    methods

        % --------------------------------------------------------------- %
        % Constructor
        % --------------------------------------------------------------- %

        function f = Filter(N,T,n)
            % Pre-allocation
            f.xf = zeros(n,N+1,T);
            f.S1 = zeros(n,N+1);
            f.S2 = zeros(1,N+1);

            % Initialize timer
            f.T_ex = 0;
            f.T_it = 0;
        end

        % --------------------------------------------------------------- %
        % Update Error Variance
        % --------------------------------------------------------------- %

        function update_error_variance(f,e,x_e)
            % Squared error of current experiment
            E = (x_e - f.xf(:,:,e)).^2;

            % Accumulate estimation error of each variable
            f.S1 = f.S1 + E;

            % Accumulate estimation error of whole state (in dB)
            f.S2 = f.S2 + 20*log10(sum(E));
        end
        
        % --------------------------------------------------------------- %
        % Compute Filter Statistics
        % --------------------------------------------------------------- %

        function compute_filter_stats(f,T,N)
            % Estimation error variance of each variable (ensemble average)
            f.S1 = f.S1 / T;

            % Estimation error variance of whole state (ensemble average)
            % f.S2 = 20*log10(sum(f.S1));
            % f.S2 = sum(f.S1);
            f.S2 = f.S2 / T;

            % Mean of estimation error variance
            f.mean_S2 = mean(f.S2);

            % Standard deviation of estimation error variance
            f.std_S2 = std(f.S2);

            % Average iteration execution time
            f.T_it = f.T_ex / (N*T);
        end
        
        % --------------------------------------------------------------- %

    end

    % =================================================================== %

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
