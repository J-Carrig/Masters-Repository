s = serialport("COM6", 2000000);
configureTerminator(s, "LF");
flush(s);

disp("Listening on COM6... Press Ctrl+C to stop")

while true
    if s.NumBytesAvailable > 0
        data = readline(s);
        fprintf("RX: %s\n", data);
    end
    pause(0.01);
end