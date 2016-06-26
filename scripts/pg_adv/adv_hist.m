%data = load('td_iw.txt');
%data = load('td_iw1.txt');
%data = load('td_iw_scaled.txt');
%data = load('dog_iw.txt');
%data = load('dog_iw_scale.txt');
%data = load('dog_iw_scale1.txt');
%data = load('dog_iw_scale3.txt');
%data = load('dog_iw_scale4.txt');
data = load('hack_iw.txt');

%data = load('cacla_adv.txt');
%data = load('td_adv.txt');
%data = load('dog_adv.txt');
%data = load('dog_adviw.txt');
%data = load('dog_adviw_scale.txt');
%data = load('dog_adviw_scale1.txt');
%data = load('hack_adv.txt');

num_bins = 100;
min_bin = 0;
max_bin = 30;
%min_bin = -3;
%max_bin = 3;

data = data .* 10;
bins = linspace(min_bin, max_bin, num_bins);
hist(data, bins);
xlim([min_bin, max_bin]);

max_val = max(data)
min_val = min(data)