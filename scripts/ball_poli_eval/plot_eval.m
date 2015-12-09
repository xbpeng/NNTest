files = {};
files{1} = 'ball_int_eval.txt';

lines = {'b-', 'r-', 'm-', 'k-'};
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
   
   xs = 1:length(curr_data);
   xs = xs .* iter_step;
   plot(xs, curr_data, line);
end
hold off;

xlabel('Iterations (10^3)');
ylabel('Success Rate');
title('Performance vs Training Iterations');

legend('Cacla Multisample');
