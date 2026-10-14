clear; clc; close all;

syms v0 a0 sf vf tf;

% We need to fine the optimal vf and tf

optimal_final_velocity = solve(diff(totalCost(v0, a0, sf, vf, 0, tf), vf) == 0, vf, 'ReturnConditions', true);
optimal_final_velocity = simplify(optimal_final_velocity.vf);

finalOptVel = matlabFunction(optimal_final_velocity, 'Vars', {v0, a0, sf, tf}, 'File', 'matlab/generated/finalOptVel');

optimal_final_time = simplify(solve(diff(totalCost(v0, a0, sf, vf, 0, tf), tf) == 0, tf));

finalOptTime = matlabFunction(optimal_final_time, 'Vars', {v0, a0, sf, vf}, 'File', 'matlab/generated/finalOptTime');

% There are conditions on vf and tf, that they need to be in an interval
% vf in [v_min, v_max]
% tf in [t_min, t_max]
% We can express the vf interval in velocity as an interval in time, using finalOptTime


