clear; clc; close all;

syms v0 a0 sf tf;

final_optimal_position_j0 = solve(jEval(0, v0, a0, sf, 0, 0, tf) == 0, sf, 'ReturnConditions', true);
final_optimal_position_j0 = final_optimal_position_j0.sf;

finalOptPosj0 = matlabFunction(final_optimal_position_j0, 'Vars', {v0, a0, tf}, 'File', 'matlab/generated/finalOptPosj0');

final_optimal_time_stop_j0 = solve(finalOptTimeStop(v0, a0, finalOptPosj0(v0, a0, tf)) == tf, tf, 'ReturnConditions', true);
final_optimal_time_stop_j0 = final_optimal_time_stop_j0.tf(2);

finalOptTimeStopj0 = matlabFunction(final_optimal_time_stop_j0, 'Vars', {v0, a0}, 'File', 'matlab/generated/finalOptTimeStopj0');
