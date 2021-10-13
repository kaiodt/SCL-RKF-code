%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                          Kalman Filter (KF)                         %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
% * Description:                                                          %
%   - Class that implements the standard Kalman Filter.                   %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef KF < Filter

    % =================================================================== %

    properties
        Pp; % Predicted estimation error weighting matrix
    end

    % =================================================================== %

    % =================================================================== %

    methods

        % --------------------------------------------------------------- %
        % Constructor
        % --------------------------------------------------------------- %

        function f = KF(N,T,n)
            % Base class constructor
            f@Filter(N,T,n);

            % Filter identification
            f.id = "KF";
        end

        % --------------------------------------------------------------- %
        % Initialization (at each new experiment)
        % --------------------------------------------------------------- %

        function initialize(f,init_params)
            % Initialize predicted estimation
            f.xp = init_params.xp0;

            % Initialize predicted estimation error weighting matrix
            f.Pp = init_params.P0;
        end

        % --------------------------------------------------------------- %
        % Update State Estimate
        % --------------------------------------------------------------- %

        function update_estimate(f,e,k,u_k,y_k,sys_model,f_params)
            % Start timer
            t_start = tic;

            % Unpack system model matrices
            F = sys_model.F;
            G = sys_model.G;
            H = sys_model.H;
            Q = sys_model.Q;
            C = sys_model.C;
            D = sys_model.D;
            R = sys_model.R;

            % Auxiliary matrices
            Qbar = H*Q*H';
            Rbar = D*R*D';

            % Kalman gain
            K = f.Pp*C'/(Rbar + C*f.Pp*C');

            % New filtered state estimate
            xf_new = f.xp + K*(y_k - C*f.xp);
            f.xf(:,k,e) = xf_new;

            % Filtered estimation error matrix
            Pf = f.Pp - K*C*f.Pp;

            % New predicted estimation error matrix
            f.Pp = Qbar + F*Pf*F';

            % New predicted state estimate
            f.xp = F*xf_new + G*u_k;

            % Stop timer and update total time
            t_iter = toc(t_start);
            f.T_ex = f.T_ex + t_iter;
        end

        % --------------------------------------------------------------- %

    end

    % =================================================================== %

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
