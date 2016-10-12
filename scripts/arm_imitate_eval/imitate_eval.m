line_width = 0.8;
files = {};

files{end + 1} = {'arm_imitate_torque_eval.txt', 'Torque', 1000};
%files{end + 1} = {'arm_imitate_mtu_eval.txt', 'MTU', 1000};
%files{end + 1} = {'arm_imitate_mtu_exp02_eval.txt', 'MTU Exp 0.2', 1000};
files{end + 1} = {'arm_imitate_mtu_exp04_eval.txt', 'MTU Exp 0.4', 1000};
files{end + 1} = {'arm_int_imitate_eval0.txt', 'Test0', 1000};
files{end + 1} = {'arm_int_imitate_eval.txt', 'Test', 1000};

lines = {'b-', 'r-', 'm-', 'k-', 'b--', 'r--', 'm--', 'k--'};

data = {};

names = {};
iter_mults = ones(length(files), 1);
max_iter = 0;

for i = 1:length(files)
   curr_data = load(files{i}{1});
   data{end + 1} = curr_data; 
   names{end + 1} = files{i}{2};
   curr_iter_mult = files{i}{3};
   iter_mults(i) = curr_iter_mult;
   max_iter = max(max_iter, curr_iter_mult * (length(data{i}) - 1));
end

clf;
hold on;
for i = 1:length(data)
   line = lines{mod((i - 1), length(lines)) + 1};
   xs = (0:(length(data{i}) - 1)) .* iter_mults(i);
   plot(xs, data{i}, line, 'LineWidth', line_width);
   
   names{i}
   data{i}(end)
end
hold off;

xlabel('Iterations');
ylabel('Average Tracking Error (m)');
title('Performance vs Training Iterations');
%xlim([0, 150000]);
%ylim([0, 1]);

legend(names);

%legend('QP', 'Reduced Torque', 'Reduced Angle', 'Reduced Velocity', 'Location', 'southeast');
%legend('QP', 'Conv Torque', 'Conv Angle', 'Conv Velocity', 'Location', 'southeast');
%legend('QP', 'Guided', 'Guided + Pretraining', 'Unguided', 'Location', 'southeast');
%legend('QP', 'With Pose', 'No Pose', 'Location', 'southeast');