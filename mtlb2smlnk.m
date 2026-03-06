Ts = 0.01;
t = (0:Ts:10)';
PWM = zeros(size(t));
PWM(t >=5.5) = 255;
PWM(t >= 1 & t < 5.5) = 128;

PWM_in = timeseries(PWM, t);
simulation = sim('Motor_Model.slx');
plot(t, simulation.RPM_out);
hold on;
yline(186/2, 'b--', 'Half RPM');
yline(186, 'r--', 'Max RPM');
