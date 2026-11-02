clear; clc; close all;

syms v0 a0 sf vf tf;

% We need to fine the optimal vf and tf

optimal_final_velocity = solve(diff(totalCost(v0, a0, sf, vf, 0, tf), vf) == 0, vf, 'ReturnConditions', true);
optimal_final_velocity = simplify(optimal_final_velocity.vf);

finalOptVel = matlabFunction(optimal_final_velocity, 'Vars', {v0, a0, sf, tf}, 'File', 'matlab/generated/finalOptVel');

% Something funky here, this may still be relevant, but the results seems correct.
optimal_final_time = solve(diff(totalCost(v0, a0, sf, vf, 0, tf), vf) == 0, tf, "ReturnConditions", true);
optimal_final_time = simplify(optimal_final_time.tf(1));

finalOptTime = matlabFunction(optimal_final_time, 'Vars', {v0, a0, sf, vf}, 'File', 'matlab/generated/finalOptTime');

% There are conditions on vf and tf, that they need to be in an interval
% vf in [v_min, v_max]
% tf in [t_min, t_max]
% We can express the vf interval in velocity as an interval in time, using finalOptTime
% These are needed to limit the vechile to drive legally and smothly.

assume(a0 < 0 & sf > 0)
T_star = solve(diff(finalOptVel(v0, a0, sf, tf), tf) == 0, tf);
T_star = T_star(isAlways(T_star > 0)); % T^* in the presentations
% The result is T_star = sqrt(15 * sf / (-a0))
T_star_calculation = matlabFunction(T_star, 'Vars', {a0, sf}, 'File', 'matlab/generated/T_star_calculation');

v_star = simplify(finalOptVel(v0, a0, sf, T_star));
% The result is v_star = (1/8) * (2 * sqrt(15 * (-a0) * sf) - 7 * v0)
v_star_calculation = matlabFunction(v_star, 'Vars', {a0, sf, v0}, 'File', 'matlab/generated/v_star_calculation');


