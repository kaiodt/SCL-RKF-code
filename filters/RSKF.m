%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                 Risk-Sensitive Kalman Filter (RSKF)                 %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
% * Description:                                                          %
%   - Class that implements the Risk-Sensitive Kalman Filter (RSKF).      %
%   - M. Zorzi. Robust Kalman Filtering Under Model Perturbations, IEEE   %
%     Trans. Automat. Control. 62 (6) (2017).                             %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef RSKF < Filter

    % =================================================================== %

    properties
        Pf; % Nominal conditional variance
        Vf; % Least-favorable conditional variance
    end

    % =================================================================== %

    % =================================================================== %

    methods

        % --------------------------------------------------------------- %
        % Constructor
        % --------------------------------------------------------------- %

        function f = RSKF(N,T,n)
            % Base class constructor
            f@Filter(N,T,n);

            % Filter identification
            f.id = "RSKF (Zorzi, 2017)";
        end

        % --------------------------------------------------------------- %
        % Initialization (at each new experiment)
        % --------------------------------------------------------------- %

        function initialize(f,init_params)
            % Initialize predicted estimation
            f.xp = init_params.xp0;

            % Initialize least-favorable conditional variance
            f.Vf = init_params.V0;
        end

        % --------------------------------------------------------------- %
        % Update State Estimate
        % --------------------------------------------------------------- %

        function update_estimate(f,e,k,u_k,y_k,sys_model,f_params)
            % Start timer
            t_start = tic;

            % Unpack system model matrices
            F = sys_model.F;
            H = sys_model.H;
            Q = sys_model.Q;
            C = sys_model.C;
            D = sys_model.D;
            R = sys_model.R;

            % Unpack filter parameters
            tau = f_params.tau;
            c   = f_params.c;

            % Dimensions
            n = size(F,1);
            p = size(H,2);
            r = size(C,1);
            q = size(D,2);

            % Adapted noise matrices
            H = [H zeros(n,q)];
            D = [zeros(r,p) D];

            % Kalman gain matrix
            K = (F*f.Vf*C' + H*D')/(C*f.Vf*C' + D*D');

            % Update filtered state estimate
            f.xf(:,k,e) = F*f.xp + K*(y_k - C*f.xp);

            % Update predicted state estimate
            f.xp = f.xf(:,k,e);

            % Update nominal conditioned variance
            f.Pf = F*f.Vf*F' - K*(C*f.Vf*C' + D*D')*K' + H*H';

            % Cholesky decomposition of Pf
            Lp = chol(f.Pf, 'lower');

            % Gamma function
            if (tau == 0)
                gamma_tau = @(th) -log(1/det(eye(n) - th*f.Pf)) + ...
                                trace(inv(eye(n) - th*f.Pf) - eye(n)) - c;
            elseif (tau == 1)
                gamma_tau = @(th) (trace(expm(th*(Lp'*Lp)) * ...
                                    (th*(Lp'*Lp - eye(n))) + eye(n)) - c);
            else
                gamma_tau = @(th) (trace(-1/(tau*(1-tau)) * ...
                (eye(n) - th*(1-tau)*(Lp'*Lp))^(tau/(tau-1)) + ...
                1/(1-tau)*(eye(n) - th*(1-tau)*(Lp'*Lp))^(1/(tau-1)) + ...
                1/tau * eye(n)) - c);
            end

            % Compute theta
            options = optimoptions('fsolve','Display','none');
            th = fsolve(gamma_tau,0.01,options);

            % Update least-favorable conditioned variance
            % f.Vf = Lp*(eye(n) - th*(1-tau)*(Lp'*Lp))^(1/(tau-1))*Lp';
            f.Vf = Lp*expm(th*(Lp'*Lp))*Lp';

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
