files = {};
files{end + 1} = {'q_learning.txt', 'Q-Learning'};
%files{end + 1} = {'cacla.txt', 'Cacla'};
files{end + 1} = {'cacla_exp_buffer.txt', 'Cacla + Exp Buffer'};
%files{end + 1} = {'cacla_exp_buffer1.txt', 'Cacla + Exp Buffer1'};
%files{end + 1} = {'cacla_exp_buffer_target.txt', 'Cacla + Exp Buffer Target'};
%files{end + 1} = {'cacla_exp_buffer_weighted.txt', 'Cacla + Exp Buffer Weighted'};
files{end + 1} = {'ace.txt', 'ACE'};
files{end + 1} = {'ace_b.txt', 'ACE b'};
%files{end + 1} = {'ace4.txt', 'ACE4'};
files{end + 1} = {'ace_init.txt', 'ACE Init'};
%files{end + 1} = {'mace.txt', 'MACE'};
%files{end + 1} = {'mace_double.txt', 'MACE Double'};
%files{end + 1} = {'mace4_double.txt', 'MACE4 Double'};

lines = {'b-', 'r-', 'm-', 'k-', 'g-', 'c-', 'k--', 'r--'};
iter_step = 500 / 1000;

data = {};
names = {};
for i = 1:length(files)
   curr_data = load(files{i}{1});
   data{i} = curr_data; 
   names{i} = files{i}{2};
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

legend(names, 'Location', 'southeast');
