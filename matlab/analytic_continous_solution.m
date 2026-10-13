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

