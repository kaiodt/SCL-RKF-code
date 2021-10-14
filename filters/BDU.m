%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                Bounded Data Uncertainty Filter (BDU)                %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
% * Description:                                                          %
%   - Class that implements the Robust Bounded Data Uncertainty Filter    %
%     (BDU).                                                              %
%   - A.H. Sayed. A Framework for State-Space Estimation with Uncertain   %
%     Models. IEEE Trans. Automat. Control. 46 (7) (2001).                %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef BDU < Filter

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

        function f = BDU(N,T,n)
            % Base class constructor
            f@Filter(N,T,n);

            % Filter identification
            f.id = "BDU (Sayed, 2001)";
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
            H  = sys_model.H;
            Q  = sys_model.Q;
            C  = sys_model.C;
            D  = sys_model.D;
            R  = sys_model.R;
            M1 = sys_model.M1;
            EF = sys_model.EF;
            EH = sys_model.EH;

            % Unpack filter parameters
            ksi = f_params.ksi;

            % Dimensions
            n = size(F,1);
            t1 = size(EF,1);

            % Lambda approximation
            lamb = (1+ksi) * norm(M1'*(C'/R*C)*M1);

            % Modified sensing model matrices
            Rhat = R - 1/lamb * C*(M1*M1')*C';
            Re = Rhat + C*f.Pp*C';

            % Filtered estimation error matrix
            Pf = f.Pp - f.Pp*C'/Re*C*f.Pp;

            % Modified system model matrices

            Qhat = inv(Q) + ...
                   lamb * EH'/(eye(t1) + lamb*EF*Pf*EF')*EH;
            
            Phat = Pf - ...
                   Pf*EF'/(1/lamb*eye(t1) + EF*Pf*EF')*EF*Pf;

            Hhat = H - lamb*F*Phat*EF'*EH;

            Fhat = (F - lamb*Hhat/Qhat*EH'*EF) * ...
                            (eye(n) - lamb*Phat*(EF'*EF));

            % New filtered state estimate
            xf_new = f.xp + Pf*C'/Rhat*(y_k - C*f.xp);
            f.xf(:,k,e) = xf_new;

            % New predicted estimation error matrix
            f.Pp = F*Phat*F' + Hhat/Qhat*Hhat';

            % New predicted state estimate
            f.xp = Fhat*xf_new;

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
