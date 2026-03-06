%% Motor & kinematics
D_wheel = 49.7;                  % mm
Kv = pi * D_wheel / 60;          % mm per second per RPM

Ts_mpc = 0.01;                    % MPC sample time

%% Error-state plant (1 state, 1 input)
A = 1;
B = -Ts_mpc * Kv;                % NEGATIVE (important!)
C = 1;
D = 0;

plant = ss(A, B, C, D, Ts_mpc);

%% MPC object (REGULATION, not tracking)
mpc_obj = mpc(plant, Ts_mpc);

mpc_obj.PredictionHorizon = 60;
mpc_obj.ControlHorizon    = 8;

%% Constraints (RPM target output of MPC)
mpc_obj.MV.Min     = 0;
mpc_obj.MV.Max     = 186;
mpc_obj.MV.RateMin = -186;
mpc_obj.MV.RateMax =  186;

%% Weights
mpc_obj.Weights.ManipulatedVariables     = 0.1;
mpc_obj.Weights.ManipulatedVariablesRate = 50;
mpc_obj.Weights.OutputVariables          = 10;

%% Nominal values (CRITICAL for explicit MPC)
mpc_obj.Model.Nominal.X = 0;   % zero distance error
mpc_obj.Model.Nominal.U = 0;
mpc_obj.Model.Nominal.Y = 0;

%% Explicit MPC ranges (ONLY STATE + MV)
range.State.Min = -1200;    % mm error
range.State.Max =  1200;

range.Reference.Min = -5;
range.Reference.Max = 5;

range.ManipulatedVariable.Min = -1;
range.ManipulatedVariable.Max = 187;

%% Generate explicit MPC
explicitMPC = generateExplicitMPC(mpc_obj, range)

%% =========================================================
%%                CLOSED-LOOP TEST SECTION
%% =========================================================

% ADDED: Simulation length
Tsim = 10;                         % seconds
N = Tsim / Ts_mpc;

% ADDED: Initial distance error (e.g. 1000 mm away)
e = zeros(N,1);
u = zeros(N,1);
e(1) = 1010;                       % initial distance error

xMPC = mpcstate(explicitMPC.MPC);
% ADDED: Closed-loop simulation
for k = 1:N-1
    
    xMPC.Plant = e(k);
    % Compute optimal RPM target from explicit MPC
    u(k) = mpcmoveExplicit(explicitMPC, xMPC);      % <--- This is the key line
    
    % Apply plant update
    e(k+1) = A*e(k) + B*u(k);
end

%% ADDED: Plot results
time = (0:N-1)*Ts_mpc;

figure;
subplot(2,1,1)
plot(time, e, 'LineWidth',2)
grid on
ylabel('Distance Error (mm)')
title('Explicit MPC Closed-Loop Test')

subplot(2,1,2)
plot(time, u, 'LineWidth',2)
grid on
ylabel('RPM Target')
%% 
xlabel('Time (s)')