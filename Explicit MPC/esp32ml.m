% MATLAB <-> ESP32 Polling/Latency Test

clear all

port = "COM9";   
baud = 2000000;

% Open serial connection
s = serialport(port, baud);

pause(1); 

numTests = 1000;
bytesToSend = uint8(55);     % send a single byte

times = zeros(1, numTests);

for i = 1:numTests
    t0 = tic;

    write(s, bytesToSend, "uint8");
    while s.NumBytesAvailable == 0
        % wait for response
    end

    data = read(s, 1, "uint8");
    times(i) = toc(t0);
end

avgTime = mean(times);
fprintf("Average round-trip time: %.6f seconds\n", avgTime);
fprintf("Equivalent polling rate: %.1f Hz\n", 1/avgTime);