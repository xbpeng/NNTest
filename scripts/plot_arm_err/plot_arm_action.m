data = load('arm_rl_action.txt');
[n, m] = size(data);

action_size = m / 2;
coach_idx = 1;
student_idx = action_size + 1;

data = data .* 10;

line_width = 1;
for i = 1:2
    subplot(2, 1, i);
    plot(data(:,coach_idx + i - 1), 'k--', 'LineWidth', line_width);
    hold on;
    plot(data(:,student_idx + i - 1), 'b-', 'LineWidth', line_width);
    hold off;

    plot_title = sprintf('Joint %i Torque', i - 1);
    title(plot_title);
    xlabel('Time (seconds)');
    ylabel('Torque (Nm)');

    legend('QP', 'Reduced Network');
end