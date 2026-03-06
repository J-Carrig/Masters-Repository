D = 49.7;
Kv = pi * D / 60;

Ts_mpc = 0.1;
nominal_target = 1000;
T = 50;
r = ones(T,1) * 1000;

A = 1;
B = Ts_mpc * Kv;
C = 1;
D = 0;

plant = ss(A, B, C, D, Ts_mpc);

mpc_obj = mpc(plant, Ts_mpc);

% Define the prediction horizon and control horizon for the MPC controller
mpc_obj.PredictionHorizon = 58;
mpc_obj.ControlHorizon = 3;

mpc_obj.MV.Min = 0;
mpc_obj.MV.Max = 186;
mpc_obj.MV.RateMin = -50;
mpc_obj.MV.RateMax = 50;

% Set the weights for the MPC controller
%mpc_obj.Weights.ManipulatedVariables = 1;
mpc_obj.Weights.ManipulatedVariablesRate = 0.1;
mpc_obj.Weights.OutputVariables = 1;

range.Reference.Min = 0;
range.Reference.Max = 8000;

range.State.Min = 0;
range.State.Max = 8000;

range.ManipulatedVariable.Min = 0;
range.ManipulatedVariable.Max = 186;

sim(mpc_obj, T, r);

explicitMPC = generateExplicitMPC(mpc_obj, range)
