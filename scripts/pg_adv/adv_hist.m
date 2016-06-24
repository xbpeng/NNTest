%data = load('cacla_adv.txt');
%data = load('td_adv.txt');
data = load('td_iw.txt');

num_bins = 100;
min_bin = -3;
max_bin = 3;

data = data .* 10;
bins = linspace(min_bin, max_bin, num_bins);
hist(data, bins);
xlim([min_bin, max_bin]);

max_val = max(data)
min_val = min(data)