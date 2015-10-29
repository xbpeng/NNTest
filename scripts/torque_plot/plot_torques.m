data = load('coach_torques.txt');

[n, m] = size(data);
data = data(:, 2:m);
[n, m] = size(data);

%hold on;
for i = 1:m
   plot(data(:, i)); 
   title('QP Torques');
   ylabel('Torques');
   xlabel('Time');
   pause;
end
%hold off;

