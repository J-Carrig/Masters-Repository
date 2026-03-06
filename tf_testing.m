s = tf('s');
K = 126;   % max RPM
tau = 0.1; % example time constant
Ts = 0.01; % 10 ms sample time

G = K/(tau*s + 1);
Gd = c2d(G, Ts, 'tustin');   % or 'zoh'
[num, den] = tfdata(Gd, 'v');  % returns vectors