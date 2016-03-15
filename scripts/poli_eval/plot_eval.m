qp_data = load('qp_eval.txt');
files = {};

%files{1} = 'nn_torque_eval.txt';
%files{2} = 'nn_pd_eval.txt';
%files{3} = 'nn_vel_eval.txt';

%files{1} = 'pixel_nn_torque_eval.txt';
%files{2} = 'pixel_nn_pd_eval.txt';
%files{3} = 'pixel_nn_vel_eval.txt';

%files{1} = 'pixel_nn_torque_eval.txt';
%files{2} = 'pixel_nn_torque_pretrain_eval.txt';
%files{3} = 'pixel_nn_torque_sync_coach_eval.txt';

%files{1} = 'pixel_nn_torque_eval.txt';
%files{2} = 'pixel_torque_no_pose.txt';

files{1} = {'gps_eval.txt', 'GPS', 1000};
files{2} = {'dmace_torque_eval.txt', 'MACE Torque', 5000};
files{3} = {'dmace_pd_eval.txt', 'MACE PD', 5000};

lines = {'b-', 'r-', 'm-', 'k-'};

data = {};

names = {};
names{1} = 'QP';
iter_mults = ones(length(files), 1);
max_iter = 0;

for i = 1:length(files)
   curr_data = load(files{i}{1});
   data{i} = curr_data; 
   names{i + 1} = files{i}{2};
   curr_iter_mult = files{i}{3};
   iter_mults(i) = curr_iter_mult;
   max_iter = max(max_iter, curr_iter_mult * (length(data{i}) - 1));
end

qp_plot_vals = [qp_data(1), qp_data(1)];

plot([0, max_iter], qp_plot_vals, 'k--');
hold on;
for i = 1:length(data)
   line = lines{mod((i - 1), length(lines)) + 1};
   xs = (0:(length(data{i}) - 1)) .* iter_mults(i);
   plot(xs, data{i}, line);
end
hold off;

xlabel('Iterations');
ylabel('Average Tracking Error (m)');
title('Performance vs Training Iterations');
%xlim([0, 50000]);
ylim([0, 1.8]);

legend(names);

%legend('QP', 'Reduced Torque', 'Reduced Angle', 'Reduced Velocity', 'Location', 'southeast');
%legend('QP', 'Conv Torque', 'Conv Angle', 'Conv Velocity', 'Location', 'southeast');
%legend('QP', 'Guided', 'Guided + Pretraining', 'Unguided', 'Location', 'southeast');
%legend('QP', 'With Pose', 'No Pose', 'Location', 'southeast');