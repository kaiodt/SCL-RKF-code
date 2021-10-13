%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                             Plot Results                            %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
% * Description:                                                          %
%   - Plots the simulation results.                                       %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% ======================================================================= %
% State estimation
% ======================================================================= %

figure();

% Experiment
e = 1;

% Final step
Nf = N;

for i = 1:n
    subplot(n,1,i);

    plot(0:Nf, x(i,1:Nf+1,e), ...
         'k-', 'LineWidth', 1.5, ...
         'DisplayName', 'Target System');

    hold on;
    grid on;

    for f = 1:n_filters
        plot(0:Nf, filters{f}.xf(i,1:Nf+1,e), ...
                'LineStyle', '--', 'LineWidth', 1.5, ...
                'DisplayName', filters{f}.id);
    end

    ylabel_str = sprintf('x_%d', i);
    ylabel(ylabel_str);
    xlabel('Time Step k');
    legend();
end

% ======================================================================= %
% Estimation error variance of each variable
% ======================================================================= %

figure();

title('Error Variance');

for i = 1:n
    subplot(n,1,i);

    for f = 1:n_filters
        plot(0:N, filters{f}.S1(i,1:N+1), ...
                'LineStyle', '-', 'LineWidth', 1.5, ...
                'DisplayName', filters{f}.id);
        grid on;
        hold on;
    end

    ylabel_str = sprintf('x_%d', i);
    ylabel(ylabel_str);
    xlabel('Time Step k');
    legend();
end

% ======================================================================= %
% Estimation error variance (dB)
% ======================================================================= %

figure();

for f = 1:n_filters
    semilogx(1:N+1, filters{f}.S2(1:N+1), ...
            'LineStyle', '-', 'LineWidth', 1.5, ...
            'DisplayName', filters{f}.id);
    grid on;
    hold on;
end

title('Error Variance');
xlabel('Time Step k');
ylabel('Error Variance (dB)');
legend();


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
