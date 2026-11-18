clear; clc; close all;

syms v0 a0 sf vf tf;

final_optimal_velocity_j0 = solve(jEval(0, v0, a0, sf, vf, 0, tf) == 0, vf, 'ReturnConditions', true);
final_optimal_velocity_j0 = final_optimal_velocity_j0.vf;

finalOptVelj0 = matlabFunction(final_optimal_velocity_j0, 'Vars', {v0, a0, sf, tf}, 'File', 'matlab/generated/finalOptVelj0');

final_optimal_time_j0 = solve(finalOptTime(v0, a0, sf, finalOptVelj0(v0, a0, sf, tf)) == tf, tf, 'ReturnConditions', true);
final_optimal_time_j0 = final_optimal_time_j0.tf(1:2);

finalOptTimej0 = matlabFunction(final_optimal_time_j0, 'Vars', {v0, a0, sf}, 'File', 'matlab/generated/finalOptTimej0');
