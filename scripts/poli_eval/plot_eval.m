qp_data = load('qp_eval.txt');
nn_torque_data = load('nn_torque_eval.txt');
nn_pd_data = load('nn_pd_eval.txt');
nn_vel_data = load('nn_vel_eval.txt');

n = length(nn_torque_data);
n = min(n, length(nn_pd_data));
n = min(n, length(nn_vel_data));

qp_plot_vals = ones(n, 1);
qp_plot_vals = qp_plot_vals .* qp_data(1);

plot(qp_plot_vals, 'k-');
hold on;
plot(nn_torque_data(1:n), 'b-');
plot(nn_pd_data(1:n), 'r-');
plot(nn_vel_data(1:n), 'g-');
hold off;

xlabel('Iterations (10^3)');
ylabel('Average Tracking Error (m)');
title('Performance vs Training Iterations');

legend('QP', 'NN Torque', 'NN PD', 'NN Vel');