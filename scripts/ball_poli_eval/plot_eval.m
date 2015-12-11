files = {};
files{1} = 'q_learning.txt';
files{2} = 'cacla.txt';
files{3} = 'cacla_exp_buffer.txt';
files{4} = 'cacla_exp_buffer_target.txt';
files{5} = 'cacla_exp_buffer_weighted.txt';
files{6} = 'cacla_big_net.txt';
files{7} = 'cacla_on_buffer.txt';

lines = {'b-', 'r-', 'm-', 'k-', 'g-', 'c-', 'k--'};
iter_step = 500 / 1000;

data = {};
for i = 1:length(files)
   curr_data = load(files{i});
   data{i} = curr_data; 
end

clf;
hold on;
for i = 1:length(data)
   curr_data = data{i};
   line = lines{mod((i - 1), length(lines)) + 1};
   
   xs = 0:(length(curr_data) - 1);
   xs = xs .* iter_step;
   plot(xs, curr_data, line);
   
   final_perf = curr_data(end);
   final_perf
end
hold off;

y_lim = get(gca, 'YLim');
y_lim(1) = 0;
ylim(y_lim);
%xlim([0, 20]);

xlabel('Iterations (10^3)');
ylabel('Success Rate');
title('Performance vs Training Iterations');

legend('Q-Learning',  'Cacla', 'Cacla Exp Buffer', ...
    'Cacla Exp Buffer + Target', 'Cacla Exp Buffer + Weighted',...
    'Cacla Big Net', 'Cacla On Buffer', 'Location', 'southeast');
