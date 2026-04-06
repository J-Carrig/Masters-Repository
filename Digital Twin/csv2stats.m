%% DT Metric Extractor & Plotter
clear; clc; close all;

% --- MANUAL CONFIGURATION ---
filename = 'PID_results_1010.csv';  % <--- Change this for each run
target_dist = 1010;             % <--- Change this to match (11, 510, or 1010)
% -----------------------------

if ~isfile(filename)
    error('File "%s" not found. Check the filename and folder.', filename);
end

% Load Data
data = readtable(filename);
t = data.Timestamp;
pwm = data.PWM;
dist = data.Distance_mm;
signal = data.Signal_995;
dt = mean(diff(t)); 

% Calculate Performance Metrics
% Rise Time (10% to 90% of Target)
idx10 = find(dist >= 0.1 * target_dist, 1);
idx90 = find(dist >= 0.9 * target_dist, 1);
if ~isempty(idx10) && ~isempty(idx90)
    rise_time = t(idx90) - t(idx10);
else
    rise_time = NaN;
end

% Settling Time (Within 0.5% of Target and stays there)
tolerance = 0.005 * target_dist;
idx_outside = find(abs(dist - target_dist) > tolerance, 1, 'last');
if isempty(idx_outside)
    settling_time = t(1); 
elseif idx_outside == length(t)
    settling_time = Inf; % Never settled within the log duration
else
    settling_time = t(idx_outside + 1);
end

% Steady State Error (Average of last 0.5s of data)
final_val = mean(dist(end-min(50, length(dist)-1):end));
ss_error = abs(target_dist - final_val);

% Control Effort (Integral of PWM)
control_effort = sum(abs(pwm)) * dt;

% Print Results to Command Window
fprintf('\n==========================================\n');
fprintf('RESULTS FOR: %s\n', filename);
fprintf('Target Distance: %.1f mm\n', target_dist);
fprintf('------------------------------------------\n');
fprintf('Rise Time (10-90%%):  %.4f seconds\n', rise_time);
fprintf('Settling Time (0.5%%):  %.4f seconds\n', settling_time);
fprintf('Steady State Error:  %.4f mm\n', ss_error);
fprintf('Total Control Effort: %.2f (PWM*s)\n', control_effort);
fprintf('==========================================\n');

% Generate the Visualization
figure('Color', 'w', 'Name', ['Analysis: ', filename]);
hold on; grid on;

% Left Y-Axis: Distance
yyaxis left
plot(t, dist, 'b-', 'LineWidth', 2.5, 'DisplayName', 'Measured Distance');
yline(target_dist, 'r--', 'Target', 'LineWidth', 1.5, 'HandleVisibility', 'off');
ylabel('Distance (mm)');
ylim([0, target_dist * 1.15]); 

% Right Y-Axis: PWM
yyaxis right
stairs(t, pwm, 'Color', [0, 0.5, 0, 0.4], 'LineWidth', 1.2, 'DisplayName', 'PWM');
ylabel('PWM (0-255)');
ylim([0, 260]);
ax = gca;
ax.YColor = [0, 0.5, 0];

% Annotate Settling Point if valid
if ~isinf(settling_time)
    yyaxis left
    plot(settling_time, final_val, 'kx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Settled');
end

xlabel('Time (seconds)');
title('Digital Twin PID Performance Analysis: 1010mm Distance');
legend('Location', 'southeast');
hold off;