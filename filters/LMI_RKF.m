%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%              LMI-Based Robust Kalman Filter (LMI-RKF)               %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
% * Description:                                                          %
%   - Class that implements the LMI-Based Robust Kalman Filter.           %
%   - M. Abolhasani, M. Rahmani. Robust Kalman Filtering for              %
%     Discrete-Time Time-Varying Systems with Stochastic and Norm-Bounded %
%     Uncertainties, J. Dyn. Syst. Meas. Control. 140 (3) (2018).         %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef LMI_RKF < Filter

    % =================================================================== %

    properties
        Pp;      % Predicted estimation error weighting matrix
        Phi_opt; % Phi matrix (optimal)
    end

    % =================================================================== %

    % =================================================================== %

    methods

        % --------------------------------------------------------------- %
        % Constructor
        % --------------------------------------------------------------- %

        function f = LMI_RKF(N,T,n)
            % Base class constructor
            f@Filter(N,T,n);

            % Filter identification
            f.id = "LMI_RKF (Abolhasani, 2018)";

            % Additional pre-allocations
            f.Phi_opt = zeros(n,n,N+1);
        end

        % --------------------------------------------------------------- %
        % Initialization (at each new experiment)
        % --------------------------------------------------------------- %

        function initialize(f,init_params)
            % Initialize predicted estimation
            f.xp = init_params.xp0;
        end

        % --------------------------------------------------------------- %
        % Pre-Compute LMIs
        % --------------------------------------------------------------- %
        
        function pre_compute_lmis(f,N,sys_model,init_params,f_params)

            fprintf("* Pre-computing LMI optimization for LMI-RKF...\n");

            % Start timer
            t_start = tic;

            % Initialize predicted estimation error weighting matrix
            f.Pp = blkdiag(init_params.P0,init_params.P0);

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
            M2 = sys_model.M2;
            EC = sys_model.EC;
            ED = sys_model.ED;

            % Unpack filter parameters
            mu = f_params.mu;
            ni = f_params.ni;

            % Dimensions
            n = size(F,1);
            p = size(H,2);
            r = size(C,1);
            q = size(D,2);
            t1 = size(EF,1);

            % Time loop
            for k = 1:N+1
                % Lambda approximation
                lamb = (1+ni) * mu * norm(M2'*M2);

                % Auxiliary matrices
                T = 1/mu * eye(r) - 1/lamb * (M2*M2');
                S = (C'/T*D + lamb*EC'*ED) / ...
                            (inv(R) + D'/T*D + lamb*(ED'*ED));

                % Augmented matrices
                M1til = [M1; M1];
                EFtil = [EF zeros(t1,n)];
                EHtil = [EH zeros(t1,q)];
                Theta = blkdiag(Q,R);

                % Scaling parameters
                alfa = (1+ni) * norm(EFtil*f.Pp*EFtil');
                beta = (1+ni) * norm(EHtil*Theta*EHtil');

                % Modified matrices
                Pptil = inv(f.Pp) - 1/alfa * (EFtil'*EFtil);
                Thtil = inv(Theta) - 1/beta * (EHtil'*EHtil);

                % Initialize YALMIP
                yalmip('clear');

                % LMI variables
                Phi = sdpvar(n);
                Pptil_1 = sdpvar(2*n);

                % Auxiliary matrices
                L = F*Phi*(C' - S*D')/T;
                Fhat = F - lamb*F*Phi*(EC' - S*ED')*EC;

                % Augmented matrices
                Ftil = [    F     zeros(n) ;
                        F - Fhat Fhat - L*C];
                Htil = [H zeros(n,q);
                        H   -L*D    ];

                % Construct LMI
                L11 = Pptil_1 - (alfa + beta)*(M1til*M1til');
                L21 = Pptil\Ftil';
                L22 = inv(Pptil);
                L31 = Thtil\Htil';
                L32 = zeros(p+q,2*n);
                L33 = inv(Thtil);

                LMI = [L11 L21' L31';
                       L21 L22  L32';
                       L31 L32  L33];

                % Aggregate LMIs
                LMIs = [LMI >= 0, Phi >= 0];

                % Define solver: SeDuMi
                opts = sdpsettings();
                opts.solver = 'sedumi';
                opts.verbose = 0;

                % Solve optimization problem subject to LMIs
                sol = optimize(LMIs, trace(Pptil_1), opts);

                % Analyze error flags
                if sol.problem == 0
                    % Extract solution
                    Phi_sol = value(Phi);
                    Pp_new  = value(Pptil_1);
                else
                    fprintf('\nLMI error!\n');
                    sol.info
                    yalmiperror(sol.problem)
                end

                % Updates
                f.Phi_opt(:,:,k) = Phi_sol;
                f.Pp = Pp_new;
            end

            % Stop timer and compute contribution to average iteration time
            t_exec = toc(t_start);
            f.T_it = t_exec / (N+1);
        end

        % --------------------------------------------------------------- %
        % Update State Estimate
        % --------------------------------------------------------------- %

        function update_estimate(f,e,k,u_k,y_k,sys_model,f_params)
            % Start timer
            t_start = tic;

            % Unpack system model matrices
            F  = sys_model.F;
            C  = sys_model.C;
            D  = sys_model.D;
            R  = sys_model.R;
            M2 = sys_model.M2;
            EC = sys_model.EC;
            ED = sys_model.ED;

            % Unpack filter parameters
            mu = f_params.mu;
            ni = f_params.ni;

            % Dimensions
            r = size(C,1);
           
            % Lambda approximation
            lamb = (1+ni) * mu * norm(M2'*M2);

            % Auxiliary matrices
            T = 1/mu * eye(r) - 1/lamb * (M2*M2');
            S = (C'/T*D + lamb*EC'*ED) / ...
                        (inv(R) + D'/T*D + lamb*(ED'*ED));

            % New filtered state estimate
            xf_new = f.xp + f.Phi_opt(:,:,k) * ...
                            (C' - S*D')/T*(y_k - C*f.xp) - ...
                            lamb*f.Phi_opt(:,:,k)*(EC' - S*ED')*EC*f.xp;
            
            f.xf(:,k,e) = xf_new;

            % New predicted state estimate
            f.xp = F*xf_new;

            % Stop timer and update total time
            t_iter = toc(t_start) + f.T_it;
            f.T_ex = f.T_ex + t_iter;
        end

        % --------------------------------------------------------------- %

    end

    % =================================================================== %

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
