%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                     Optimal Robust Filter (ORF)                     %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
% * Description:                                                          %
%   - Class that implements the Optimal Robust Filter.                    %
%   - J.Y. Ishihara, M.H. Terra, J.P. Cerri. Optimal Robust Filtering for %
%     Systems Subject to Uncertainties. Automatica. 52 (2015).            %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef ORF < Filter

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

        function f = ORF(N,T,n)
            % Base class constructor
            f@Filter(N,T,n);

            % Filter identification
            f.id = "ORF (Ishihara, 2015)";
        end

        % --------------------------------------------------------------- %
        % Initialization (at each new experiment)
        % --------------------------------------------------------------- %

        function initialize(f,init_params)
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

            if (k == 1)
                % Initial filtered estimation error matrix
                f.Pf = inv(inv(f.Pf) + C'/R*C);

                % Initial filtered state estimate
                f.xf(:,1,e) = f.Pf*C'/R*y_k;
            else
                % Unpack filter parameters
                mu  = f_params.mu;
                ksi = f_params.ksi;

                % Dimensions
                n = size(F,1);
                r = size(C,1);
                p = size(H,2);
                q = size(D,2);
                t1 = size(EF,1);
                t2 = size(EC,1);

                % Lambda approximation
                % lamb = (1+ksi) * mu * norm(blkdiag((M1'*M1), (M2'*M2)));

                % Auxiliary matrices (using parameters)
                % Phi1 = 1/mu * eye(n) - 1/lamb * (M1*M1');
                % Phi2 = 1/mu * eye(r) - 1/lamb * (M2*M2');
                % Lam = 1/lamb * eye(t1+t2);

                % Auxiliary matrices (without parameters)
                Phi1 = 1/mu * eye(n);
                Phi2 = 1/mu * eye(r);
                Lam = 1/mu * eye(t1+t2);

                Pbar = blkdiag(f.Pf, Q, R);
                Phi = blkdiag(Phi1, Phi2);

                Ii = [eye(n+p+q) zeros(n+p+q,n)];

                A = [     F          H     zeros(n,q) -eye(n);
                     zeros(r,n) zeros(r,p)      D        C   ];

                EA = [    EF          EH      zeros(t1,q) zeros(t1,n);
                      zeros(t2,n) zeros(t2,p)     ED          EC     ];

                x = [f.xf(:,k-1,e) ;
                     zeros(p+q,1)];

                b = [-G * u_k;
                        y_k  ];

                Eb = [ -EG * u_k ;
                      zeros(t2,1)];

                % Framework matrices

                Pcal = blkdiag(Pbar, Phi, Lam);

                Acal = [Ii; A; EA];

                bcal = [x; b; Eb];

                M = [Pcal       Acal     ;
                     Acal' zeros(2*n+p+q)];

                v = [    bcal       zeros(size(bcal,1),n);
                     zeros(n+p+q,1)     zeros(n+p+q,n)   ;
                      zeros(n,1)           -eye(n)       ];

                u = [zeros(size(bcal,1),n);
                         zeros(n+p+q,n)   ;
                            eye(n)        ]; 

                % Solution
                X = u' * (M \ v);

                % New filtered state estimate
                f.xf(:,k,e) = X(:,1);

                % New filtered estimation error matrix
                f.Pf = X(:,2:n+1);
            end

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
