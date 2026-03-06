% serial communication script between MATLAB and ESP32

port = 'COM9';
baud = 2000000;

s = serialport(port, baud);
configureTerminator(s, 'LF');
setDTR(s, false);
setRTS(s,false);
pause(2)
flush(s);

distance_target = 250;
writeline(s, num2str(distance_target));

disp("Target dsitance sent");
figure
h = animatedline;
xlabel('Time (S)');
ylabel('Value');
grid on;

t0 = tic;

disp("Reading from ESP32... Press Ctrl+C to stop");

while true
    if s.NumBytesAvailable > 0
        line = readline(s);
        data = str2double(split(line, ","));

        if numel(data) == 2 && all(~isnan(data))
            rpm = data(1);
            distance = data(2);

            t = toc(t0);
            addpoints(h, t, distance);
            drawnow limitrate;

            fprintf("RPM = %.2f | Distance = %.2f\n", rpm, distance )

        else
            fprintf("ESP32 echoed target: %s\n", line);
        end
    end
end



