%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                      Robust Kalman Filter (RKF)                     %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
% * Description:                                                          %
%   - Class that implements the Robust Kalman Filter.                     %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef RKF < Filter

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

        function f = RKF(N,T,n)
            % Base class constructor
            f@Filter(N,T,n);

            % Filter identification
            f.id = "RKF";
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
            F  = sys_model.F;
            G  = sys_model.G;
            H  = sys_model.H;
            Q  = sys_model.Q;
            C  = sys_model.C;
            D  = sys_model.D;
            R  = sys_model.R;
            M1 = sys_model.M1;
            EF = sys_model.EF;
            EG = sys_model.EG;
            EH = sys_model.EH;
            M2 = sys_model.M2;
            EC = sys_model.EC;
            ED = sys_model.ED;

            % Unpack filter parameters
            mu  = f_params.mu;
            ksi = f_params.ksi;

            % Dimensions
            n = size(F,1);
            r = size(C,1);
            t1 = size(EF,1);
            t2 = size(EC,1);

            % Check if there are uncertainties in the sensing model
            if (all(M2 == 0))
                % Lambda approximation
                lamb = (1+ksi) * mu * norm(M1'*M1);

                % Modified sensing model matrices
                Phi2 = 1/mu * eye(r);
                Rhat = Phi2 + D*R*D';
                Chat = C;
            else
                % Lambda approximation
                lamb = (1+ksi) * mu * norm(blkdiag((M1'*M1), (M2'*M2)));

                % Modified sensing model matrices
                Phi2 = 1/mu * eye(r) - 1/lamb * (M2*M2');
                Rbar = 1/lamb * eye(t2) + ED*R*ED';
                Rhat = Phi2 + D/(inv(R) + lamb*(ED'*ED))*D';
                Chat = C - D*R*ED'/Rbar*EC;
            end

            % Modified system model matrices
            Phi1 = 1/mu * eye(n) - 1/lamb * (M1*M1');
            Qbar = 1/lamb * eye(t1) + EH*Q*EH';
            Qhat = Phi1 + H/(inv(Q) + lamb*(EH'*EH))*H';
            Fhat = F - H*Q*EH'/Qbar*EF;
            Ghat = G - H*Q*EH'/Qbar*EG;

            % Filtered estimation error matrix (inverse)
            if (all(M2 == 0))
                Pf_i = inv(f.Pp) + EF'/Qbar*EF + Chat'/Rhat*Chat;
            else
                Pf_i = inv(f.Pp) + EF'/Qbar*EF + ...
                                        Chat'/Rhat*Chat + EC'/Rbar*EC;
            end

            % New filtered state estimate
            xf_new = Pf_i \ ...
                        (f.Pp\f.xp + Chat'/Rhat*y_k - EF'/Qbar*EG*u_k);

            f.xf(:,k,e) = xf_new;

            % New predicted estimation error matrix
            f.Pp = Fhat/Pf_i*Fhat' + Qhat;

            % New predicted state estimate
            f.xp = Fhat*xf_new + Ghat*u_k;

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
