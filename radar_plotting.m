%PID vs MPC Radar Comparison (1010mm) - 3 Separate Figures
%% data matrix
dt_pid  = [2.48,   4.07,   0.452,  785.83];
dt_mpc  = [2.46,   4.26,   0.416,  785.84];
hil_pid = [2.10,   3.91,   1.842,  543.36];
hil_mpc = [2.54,   5.24,   2.194,  529.96];
hw_pid  = [1.69,   3.05,   0.053,  580.68];
hw_mpc  = [2.10,   3.39,   1.769,  555.48];

%% normalisation
all_data = [dt_pid; dt_mpc; hil_pid; hil_mpc; hw_pid; hw_mpc];
max_vals = max(all_data); 

n_dt_pid  = dt_pid  ./ max_vals;
n_dt_mpc  = dt_mpc  ./ max_vals;
n_hil_pid = hil_pid ./ max_vals;
n_hil_mpc = hil_mpc ./ max_vals;
n_hw_pid  = hw_pid  ./ max_vals;
n_hw_mpc  = hw_mpc  ./ max_vals;

%% Plotting
labels = {'Rise Time', 'Settling Time', 'SS Error', 'Effort'};
theta = linspace(0, 2*pi, length(labels) + 1);
env_names = {'Digital Twin (1010mm)', 'HIL Simulation (1010mm)', 'Full Hardware (1010mm)'};
pid_sets = {n_dt_pid, n_hil_pid, n_hw_pid};
mpc_sets = {n_dt_mpc, n_hil_mpc, n_hw_mpc};

% Loop through and create 3 separate figures
for i = 1:3
    figure('Color', 'w', 'Name', env_names{i});
    
    % Prepare data for circular plot (close the loop)
    p_data = [pid_sets{i}, pid_sets{i}(1)];
    m_data = [mpc_sets{i}, mpc_sets{i}(1)];
    
    % Plot PID (Red)
    polarplot(theta, p_data, 'r-o', 'LineWidth', 2.5, 'MarkerFaceColor', 'r');
    hold on;
    
    % Plot MPC (Blue)
    polarplot(theta, m_data, 'b-s', 'LineWidth', 2.5, 'MarkerFaceColor', 'b');
    
    % Formatting the circular grid (Rings)
    ax = gca;
    ax.ThetaTick = rad2deg(theta(1:end-1));
    ax.ThetaTickLabel = labels;
    
    % Set the specific rings
    rticks([0 0.2 0.4 0.6 0.8 1.0]); 
    rticklabels({'0', '0.2', '0.4', '0.6', '0.8', '1.0'});
    rlim([0 1.1]); % Slightly more than 1 to keep labels from clipping
    
    title(env_names{i}, 'FontSize', 14, 'FontWeight', 'bold');
    legend('PID', 'MPC', 'Location', 'northeastoutside');
    
    % Make the grid lines subtle but visible
    ax.GridColor = [0.1, 0.1, 0.1];
    ax.GridAlpha = 0.15;
end