%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                    Guaranteed Cost Filter (GCF)                     %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
% * Description:                                                          %
%   - Class that implements the Guaranteed Cost Filter (GCF).             %
%   - Z. Dong, Z. You. Finite-Horizon Robust Kalman Filtering for         %
%     Uncertain Discrete Time-Varying Systems with Uncertain-Covariance   %
%     White Noises. IEEE Signal Process. Lett. 13 (8) (2006).             %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef GCF < Filter

    % =================================================================== %

    properties
        Pf; % Filtered estimation error weighting matrix
    end

    % =================================================================== %

    % =================================================================== %

    methods

        % --------------------------------------------------------------- %
        % Constructor
        % --------------------------------------------------------------- %

        function f = GCF(N,T,n)
            % Base class constructor
            f@Filter(N,T,n);

            % Filter identification
            f.id = "GCF (Dong, 2006)";
        end

        % --------------------------------------------------------------- %
        % Initialization (at each new experiment)
        % --------------------------------------------------------------- %

        function initialize(f,init_params)
            % Initialize predicted estimation
            f.xp = init_params.xp0;

            % Initialize filtered estimation error weighting matrix
            f.Pf = init_params.P0;
        end

        % --------------------------------------------------------------- %
        % Update State Estimate
        % --------------------------------------------------------------- %

        function update_estimate(f,e,k,u_k,y_k,sys_model,f_params)
            % Start timer
            t_start = tic;

            % Unpack system model matrices
            F  = sys_model.F;
            H  = sys_model.H;
            Q  = sys_model.Q;
            C  = sys_model.C;
            D  = sys_model.D;
            R  = sys_model.R;
            M1 = sys_model.M1;
            M2 = sys_model.M2;
            E1 = sys_model.EF;
            E2 = sys_model.EH;

            % Unpack filter parameters
            eps_a = f_params.eps_a;
            eps_b = f_params.eps_b;

            % Dimensions
            n  = size(F,1);
            t1 = size(E1,1);
            t2 = size(E2,1);
            
            % Scaling parameters
            alfa = norm(E1*f.Pf*E1') + eps_a;
            beta = norm(E2*Q*E2') + eps_b;

            % Modified matrices

            Pbar = f.Pf + f.Pf*E1'/(alfa*eye(t1) - E1*f.Pf*E1')*E1*f.Pf;

            Qbar = Q + Q*E2'/(beta*eye(t2) - E2*Q*E2')*E2*Q;
            
            Rbar = R + R*E2'/(beta*eye(t2) - E2*R*E2')*E2*R;

            % Filter gain
            K = (F*Pbar*C' + alfa*(M1*M2')) / ...
                    (C*Pbar*C' + D*Rbar*D' + (alfa + beta)*(M2*M2'));

            % New filtered state estimate
            f.xf(:,k,e) = F*f.xp + K*(y_k - C*f.xp);

            % New error weighting matrix
            f.Pf = F*f.Pf*F' + H*Qbar*H' - K*(C*Pbar*F' + alfa*(M2*M1'));

            % New predicted state estimate
            f.xp = (eye(n) + f.Pf*E1' / ...
                    (alfa*eye(t1) - E1*f.Pf*E1')*E1)*f.xf(:,k,e);

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
