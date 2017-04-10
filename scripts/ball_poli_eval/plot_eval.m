files = {};
%files{end + 1} = {'q_learning.txt', 'Q-Learning'};

%files{end + 1} = {'cacla0.txt', 'Cacla 0'};
%files{end + 1} = {'cacla1.txt', 'Cacla 1'};
%files{end + 1} = {'cacla2.txt', 'Cacla 2'};
%files{end + 1} = {'cacla3.txt', 'Cacla 3'};
%files{end + 1} = {'cacla4.txt', 'Cacla 4'};
%files{end + 1} = {'cacla5.txt', 'Cacla 5'};
%files{end + 1} = {'cacla6.txt', 'Cacla'};

%files{end + 1} = {'cacla_net1.txt', 'Cacla'};
files{end + 1} = {'cacla_net1_025exp.txt', 'Cacla, 0.25 exp'};

%files{end + 1} = {'cacla_off_poli.txt', 'Cacla Off Policy'};

%files{end + 1} = {'pg_off_poli.txt', 'PG Off Policy'};
%files{end + 1} = {'pg_off_poli1.txt', 'PG Off Policy1'};
%files{end + 1} = {'pg_on_poli.txt', 'PG On Policy'};
%files{end + 1} = {'pg_no_is.txt', 'PG No IS'};
%files{end + 1} = {'pg_trunc_iw.txt', 'PG Trunc IW'};
%files{end + 1} = {'pg_off_poli_trunc_20.txt', 'PG Trunc 20'};
%files{end + 1} = {'pg_off_poli_trunc_2.txt', 'PG Trunc 2'};

%files{end + 1} = {'td_10.txt', 'PG No Importance Sampling'};

%files{end + 1} = {'cacla_stochastic0.txt', 'Cacla Stochastic0'};
%files{end + 1} = {'cacla_stochastic1.txt', 'Cacla Stochastic, 0.5 action noise, 0.2 state noise'};
files{end + 1} = {'cacla_stochastic2.txt', 'Cacla Stochastic (Reparameterization Trick)'};
%files{end + 1} = {'cacla_stochastic_no_state_noise.txt', 'Cacla Stochastic No State Noise'};

%files{end + 1} = {'cacla_param_noise0.txt', 'Cacla Stochastic (Parameteric Noise)'};
files{end + 1} = {'cacla_param_noise1.txt', 'Cacla Stochastic Net3 (Parameteric Noise)'};

files{end + 1} = {'ball_int_eval.txt', 'Test'};

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
xlim([0, 100]);

xlabel('Iterations (10^3)');
ylabel('Success Rate');
title('Performance vs Training Iterations');

legend(names, 'Location', 'southeast');
