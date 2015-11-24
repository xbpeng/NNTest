qp_data = load('qp_eval.txt');
files = {};

%files{1} = 'nn_torque_eval.txt';
%files{2} = 'nn_pd_eval.txt';
%files{3} = 'nn_vel_eval.txt';

files{1} = 'pixel_nn_torque_eval.txt';
files{2} = 'pixel_nn_pd_eval.txt';
files{3} = 'pixel_nn_vel_eval.txt';
%files{4} = 'pixel_nn_torque_pretrain_eval.txt';

%files{1} = 'pixel_nn_torque_eval.txt';
%files{2} = 'pixel_nn_torque_no_replay_eval.txt';
%files{3} = 'pixel_nn_torque_sync_coach_eval.txt';

lines = {'b-', 'r-', 'm-', 'k-'};

data = {};

n = inf;
n = 100;
for i = 1:length(files)
   curr_data = load(files{i});
   data{i} = curr_data; 
   n = min(n, length(curr_data));
end

qp_plot_vals = ones(n, 1);
qp_plot_vals = qp_plot_vals .* qp_data(1);

for i = 1:length(data)
   data{i} = data{i}(1:n);
end

plot(qp_plot_vals, 'k--');
hold on;
for i = 1:length(data)
   line = lines{mod((i - 1), length(lines)) + 1};
   plot(data{i}, line);
end
hold off;

xlabel('Iterations (10^3)');
ylabel('Average Tracking Error (m)');
title('Performance vs Training Iterations');
xlim([0, 90]);
ylim([0.4, 2]);

%legend('QP', 'Reduced Torque', 'Reduced Angle', 'Reduced Velocity');
legend('QP', 'Conv Torque', 'Conv Angle', 'Conv Velocity');
%legend('QP', 'Replay + Guided', 'Guided', 'Replay');