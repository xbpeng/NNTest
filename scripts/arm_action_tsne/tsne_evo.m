addpath 'C:\Users\Jason\Documents\compsci_stuff\matlab\tsne';

action_dir = 'action_data\';
iter_mult = 10000;

dir_list = dir(action_dir);
files = {};
for i = 1:length(dir_list)
    f_name = dir_list(i).name; 
    if (~strcmp(f_name, '.') && ~strcmp(f_name, '..'))
        files{end + 1} = f_name;
        f_name
    end
end

num_files = length(files);

colors = [0, 0, 1;
          1, 0, 0;
          0, 0.75, 0;
          0.75, 0, 0.75;
          0, 0.5, 0.5;
          0, 0, 0];
num_cols = size(colors, 1);

id_col = 1;
param_start = 2;

beg_indices = zeros(1, num_files);
end_indices = zeros(1, num_files);

data = [];
for f = 1:num_files
    curr_file = strcat(action_dir, files{f});
    curr_data = load(curr_file);
    
    if (f == 1)
       data = curr_data(1, :);
    end
    
    [curr_rows, curr_cols] = size(data);
    
    if (f == 1)
       curr_beg_idx = 1; 
    else
        curr_beg_idx = curr_rows + 1;
    end
    
    data = [data; curr_data];
    
    [curr_rows, curr_cols] = size(data);
    curr_end_idx = curr_rows;
    
    beg_indices(f) = curr_beg_idx;
    end_indices(f) = curr_end_idx;
end

ids = data(:, id_col);
actions = data(:, param_start:end);

[m, n] = size(actions);
action_mean = mean(actions, 1);
action_stdev = std(actions, 1);
for i = 1:m
   actions(i,:) = (actions(i,:) - action_mean) ./ action_stdev; 
end

num_pts = m;
num_pts

new_ids = ids;
labels = unique(new_ids);
num_labels = length(labels);

actor_freqs = zeros(num_labels, num_files);
for i = 1:num_files
    curr_beg = beg_indices(i);
    curr_end = end_indices(i);
    
    iter_ids = new_ids(curr_beg:curr_end);
    total_steps = length(iter_ids);
    
    for a = 1:num_labels
        curr_label = labels(a);

        curr_indices = (iter_ids == curr_label);
        active_steps = nnz(curr_indices);
        actor_freqs(a, i) = active_steps / total_steps;
    end
end

% plot frequencies
figure(1);
clf();
hold on;
xs = (1:num_files) - 1;
xs = xs .* iter_mult;
for a = 1:num_labels
    col_idx = mod(a - 1, num_cols) + 1;
    curr_col = colors(col_idx, :);
    curr_freqs = actor_freqs(a,:);
    plot(xs, curr_freqs, '-', 'Color', curr_col);
end
hold off;

% t-sne!!
num_dims = 2;
num_features = n;
initial_dims = num_features;
perplexity = 30;
theta = 0.5;

mapped_pts = fast_tsne(actions, num_dims, initial_dims, perplexity, theta);

% plot
figure(3);
clf;
axis off;

pad = 1.1;
min_x = min(mapped_pts(:,1)) * pad;
max_x = max(mapped_pts(:,1)) * pad;
min_y = min(mapped_pts(:,2)) * pad;
max_y = max(mapped_pts(:,2)) * pad;

mapped_new_actions = mapped_pts;


for i = 1:num_files
    curr_beg = beg_indices(i);
    curr_end = end_indices(i);
    
    iter_ids = new_ids(curr_beg:curr_end);
    iter_mapped_actions = mapped_new_actions(curr_beg:curr_end, :);
    
    clf;
    hold on
    names = {};
    
    for a = 1:num_labels
        curr_label = labels(a);

        curr_indices = (iter_ids == curr_label);
        curr_pts = iter_mapped_actions(curr_indices, :);
        [curr_m, curr_n] = size(curr_pts);
        
        names{end + 1} = sprintf('Actor %i', curr_label);
        
        if (curr_m > 0)
            col_idx = mod(a - 1, num_cols) + 1;
            curr_col = colors(col_idx, :);
            plot(curr_pts(:,1), curr_pts(:,2), '.', 'Color', curr_col);
        end
    end
    
    hold off
    
    xlim([min_x, max_x]);
    ylim([min_y, max_y]);
    set(gca,'YTick',[]);
    set(gca,'XTick',[]);
    
    curr_iter = (i - 1) * iter_mult;
    title_str = sprintf('Iteration %i', curr_iter);
    title(title_str);
    legend(names, 'Interpreter', 'none');
    
    out_file = strcat('out_plots/actions_', sprintf('%04d',i));
    print(out_file, '-dpng');
    pause;
end
