qp_data = load('qp_eval.txt');
%nn_torque_data = load('nn_torque_eval.txt');
%nn_pd_data = load('nn_pd_eval.txt');
%nn_vel_data = load('nn_vel_eval.txt');

nn_torque_data = load('pixel_nn_torque_eval.txt');
nn_torque_pretrain_data = load('pixel_nn_torque_pretrain_eval.txt');
nn_pd_data = load('pixel_nn_pd_eval.txt');
nn_vel_data = load('pixel_nn_vel_eval.txt');

n = length(nn_torque_data);
n = min(n, length(nn_torque_pretrain_data));
n = min(n, length(nn_pd_data));
n = min(n, length(nn_vel_data));
n = min(100, n);

qp_plot_vals = ones(n, 1);
qp_plot_vals = qp_plot_vals .* qp_data(1);

nn_torque_data = nn_torque_data(1:n);
nn_torque_pretrain_data = nn_torque_pretrain_data(1:n);
nn_pd_data = nn_pd_data(1:n);
nn_vel_data = nn_vel_data(1:n);

plot(qp_plot_vals, 'k--');
hold on;
plot(nn_torque_data, 'b-');
plot(nn_torque_pretrain_data, 'm-');
plot(nn_pd_data, 'r-');
plot(nn_vel_data, 'k-');
hold off;

xlabel('Iterations (10^3)');
ylabel('Average Tracking Error (m)');
title('Performance vs Training Iterations');

%legend('QP', 'NN Torque', 'NN Angle', 'NN Velocity');
legend('QP', 'NN Torque', 'NN Torque Pretrain', 'NN Angle', 'NN Velocity');