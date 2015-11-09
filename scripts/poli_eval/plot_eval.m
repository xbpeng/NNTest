qp_data = load('qp_eval.txt');
nn_torque_data = load('nn_torque_eval.txt');

n = length(nn_torque_data);

qp_plot_vals = ones(n, 1);
qp_plot_vals = qp_plot_vals .* qp_data(1);

plot(qp_plot_vals, 'k-');
hold on;
plot(nn_torque_data, 'b-');
hold off;

xlabel('Iterations');
ylabel('Tracking Error');
title('Performance vs Training Iterations');

legend('QP', 'NN Torque');