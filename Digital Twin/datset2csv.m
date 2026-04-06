% Extract the specific signals from the Dataset
% use .Values to get the timeseries data
dist_ts    = out.ScopeData.get('distance').Values;
pwm_ts     = out.ScopeData.get('PWM').Values;
reached_ts = out.ScopeData.get('has_reached distance').Values;

% Create a Table
% use dist_ts.Time to ensure we have the correct timestamps
T = table(dist_ts.Time, ...
          pwm_ts.Data, ...
          reached_ts.Data, ...
          dist_ts.Data, ...
          'VariableNames', {'Timestamp', 'PWM', 'Signal_995', 'Distance_mm'});

% Save to CSV
writetable(T, 'sim_results_1010mm.csv');

fprintf('Successfully exported simulation data to sim_results_1010mm.csv\n');

HIL PID Performance: 1010mm Step Response