def_optimal_control

odes
conds
solution

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
