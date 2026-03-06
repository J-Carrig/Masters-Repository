try
    s = serialport("COM9",115200)
    disp("Connected!");
catch ME
    disp(ME.message)
end
