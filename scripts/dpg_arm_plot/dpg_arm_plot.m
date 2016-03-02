critic_data = load('critic_vals.txt');
actor_data = load('actor_data.txt');
dpg_data = load('dpg_data.txt');

subplot(3, 1, 1);
plot(critic_data(:,1), critic_data(:,2));
title('Critic Vals');

subplot(3, 1, 2);
plot(actor_data(:,1), actor_data(:,2));
title('Actor Policy');

subplot(3, 1, 3);
plot(dpg_data(:,1), dpg_data(:,2));
title('DPG Data');