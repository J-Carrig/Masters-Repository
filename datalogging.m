clear; clc; close all;

% ==============================
% Configuration
% ==============================
% Set this to the port used by Serial.begin(115200)
matlabPort = "COM7"; 
baudRate = 115200;

targetDistance = 1010.0; 
thresholdDist = targetDistance * 0.995; % 1004.95 mm

% ==============================
% Setup Serial Connection
% ==============================
if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

try
    device = serialport(matlabPort, baudRate);
    configureTerminator(device, "CR/LF");
    setDTR(device, false);
    setRTS(device,false);
    flush(device);
    fprintf('Connected to %s successfully.\n', matlabPort);
catch
    error('Could not open %s. Check your COM port number and ensure the IDE Serial Monitor is closed.', matlabPort);
end

% ==============================
% Setup Plots
% ==============================
fig = figure('Name', 'HIL Telemetry - MATLAB Side', 'Color', 'w', 'Position', [100, 100, 800, 800]);

% Distance Plot
ax1 = subplot(3,1,1);
hDist = animatedline('Color', '#0072BD', 'LineWidth', 1.5);
yline(targetDistance, '--r', 'Target', 'LabelHorizontalAlignment', 'left');
title('Plant Position: Distance (mm)');
grid on; ylabel('mm');

% RPM Plot
ax2 = subplot(3,1,2);
hRPM = animatedline('Color', '#D95319', 'LineWidth', 1.5);
title('Plant Velocity: RPM');
grid on; ylabel('RPM');

% PWM & Threshold Signal
ax3 = subplot(3,1,3);
yyaxis left
hPWM = animatedline('Color', '#77AC30', 'LineWidth', 1.5);
ylabel('PWM (0-255)');
ylim([-10 265]);

yyaxis right
hSignal = animatedline('Color', '#A2142F', 'LineWidth', 2, 'LineStyle', '-');
ylabel('Status Signal');
ylim([-5 110]); % Keep it 0 or 100
legend([hPWM, hSignal], {'PWM Output', 'Distance > 99.5%'}, 'Location', 'southoutside', 'Orientation', 'horizontal');

title('Controller Output & Target Logic');
grid on; xlabel('Time (seconds)');

% ==============================
% Live Execution Loop
% ==============================
startTime = datetime('now');
cleanupObj = onCleanup(@() clear('device')); 

while ishandle(fig)
        lineData = readline(device);
        vals = str2double(split(lineData, ','));
        
        if length(vals) == 3
            dist_val = vals(1);
            rpm_val  = vals(2);
            pwm_val  = vals(3);
            
            % Logic for the 99.5% Signal
            if dist_val >= thresholdDist
                sig_99 = 100;
            else
                sig_99 = 0;
            end
            
            % Time tracking
            t = seconds(datetime('now') - startTime);
            
            % Add data to lines
            addpoints(hDist, t, dist_val);
            addpoints(hRPM, t, rpm_val);
            addpoints(hPWM, t, pwm_val);
            addpoints(hSignal, t, sig_99);
            
            % Dynamic Axis Scaling (Scroll effect after 10 seconds)
            if t > 10
                ax1.XLim = [t-10 t];
                ax2.XLim = [t-10 t];
                ax3.XLim = [t-10 t];
            end
            
            drawnow limitrate;
        end
end