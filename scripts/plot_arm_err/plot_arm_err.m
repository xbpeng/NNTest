files = {};
files{1} = 'qp_err.txt';
files{2} = 'nn_err.txt';
files{3} = 'nn_pixel_err.txt';

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

line_width = 1;

clf();
hold on;
for i = 1:length(data) 
    line = lines{mod(i - 1, length(lines)) + 1};

    curr_data = data{i};
    curr_err = sqrt(sum(curr_data(:,1).^2, 2));
    plot(curr_err, line, 'LineWidth', line_width);
end
hold off;

title('Tracking Error');
xlabel('Time (seconds)');
ylabel('Tracking Error (m)');

legend('QP', 'Reduced Network', 'Conv Network');