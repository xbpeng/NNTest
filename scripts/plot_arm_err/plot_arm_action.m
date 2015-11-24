files = {};
files{1} = 'qp_action.txt';
files{2} = 'nn_action.txt';
files{3} = 'nn_pixel_action.txt';

lines = {'k--', 'b-', 'r-'};

data = {};
n = inf;
m = 0;
for i = 1:length(files)
    curr_data = load(files{i});
    [curr_n, curr_m] = size(curr_data);
    n = min(curr_n, n);
    m = curr_m;
    
    data{i} = curr_data;
end

for i = 1:length(data)
    data{i} = data{i}(1:n, 1:m);
end

action_mult = 100;
line_width = 1;

for j = 1:m
    clf();
    hold on;
    for i = 1:length(data) 
        line = lines{mod(i - 1, length(lines)) + 1};

        curr_data = data{i};
        plot(curr_data(:,j) .* action_mult, line, 'LineWidth', line_width);
    end
    hold off;
    
    plot_title = sprintf('Joint %i Torque', j - 1);
    title(plot_title);
    xlabel('Time (seconds)');
    ylabel('Torque (Nm)');
    legend('QP', 'Reduced Network', 'Conv Network');
    
    pause;
end
