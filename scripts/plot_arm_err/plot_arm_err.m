data = load('arm_rl_err.txt');
data = data .* data;

[n, m] = size(data);
time = 1:n;
time = (time - 1) .* (1 / 30);

coach_err = sqrt(data(:,1) + data(:,2));
student_err = sqrt(data(:,3) + data(:,4));

plot(time, coach_err, 'k--');
hold on;
plot(time, student_err, 'b-');
hold off;

xlim([0, 2.5]);

title('Tracking Error');
xlabel('Time (seconds)');
ylabel('Tracking Error (m)');

legend('QP', 'Reduced Network');