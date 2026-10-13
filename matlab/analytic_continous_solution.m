clear; clc; close all;

% Define the plant
syms s(t) v(t) a(t) j(t);
x = [s; v; a];
u = j;

A = [
    0, 1, 0;
    0, 0, 1;
    0, 0, 0;
];
B = [0; 0; 1];

f = A * x + B * u;
% The plant is described by: diff(x) == f;

% Define the Lagranigan multipliers
syms l1(t) l2(t) l3(t);
l = [l1; l2; l3];

% The Lagrangian = the running cost
L = j^2;

% The hamiltonian = Previously know as the lagrangian = the objective function + the constraints
H = l.' * f + L;

% Fining the minimum of the Hamiltonian
H_eq = functionalDerivative(H, u) == 0;
u_opt = rhs(isolate(H_eq, u(t)));
f_opt = A * x + B * u_opt;

% Then add the second optimality condition

odes = [
    diff(x) == f_opt;
    diff(l) == -functionalDerivative(H, x);
]

% boundary conditions
syms v0 a0 sf vf af tf;
conds = [
    x(0) == [0; v0; a0];
    x(tf) == [sf; vf; af]
]

solution = dsolve(odes, conds)
j_solution = subs(u_opt, l3(t), solution.l3);
opt_fun = matlabFunction([solution.s; solution.v; solution.a; j_solution], 'Vars', {t, v0, a0, sf, vf, af, tf});

%% Plot the results

tf_val = 4.;
t_val = linspace(0, tf_val, 200);
v0_val = 10;
a0_val = 1;
sf_val = 90;
vf_val = 25;
af_val = 0.;

values = opt_fun(t_val, v0_val, a0_val, sf_val, vf_val, af_val, tf_val);

figure(1);

figure(1);
titles = {'Position', 'Velocity', 'Acceleration', 'Jerk'};
ylabels = {'s [m]', 'v [m/s]', 'a [m/s^2]', 'j [m/s^3]'};

for k = 1:4
    subplot(2, 2, k);
    plot(t_val, values(k, :));
    title(titles{k});
    xlabel('Time [s]');
    ylabel(ylabels{k});
end

sgtitle('Minimum-jerk trajectory');
