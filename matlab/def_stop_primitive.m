clear; clc; close all;

syms v0 a0 sf tf;

optimal_stop_time = solve(diff(totalCost(v0, a0, sf, 0, 0, tf), tf) == 0, tf);
optimal_stop_time = optimal_stop_time(2); % Only keeping the seconds solution, as the other gives a negative time.
% This expression is formatted different from the slides, but it is the same

finalOptTimeStop = matlabFunction(optimal_stop_time, 'Vars', {v0, a0, sf}, 'File', 'matlab/generated/finalOptTimeStop');

% Inspecting the optimal_stop_time we have a root
% sqrt(4 * v0^2 + 5 * a0 * sf)
% Assuming that v0, a0 and sf are real, to have a real solution in the case of a0 < 0,
% then sf <= -(4 * v0^2) / (5 * a0)
% This means that given a0 and v0, there is a maximum reachable position.
