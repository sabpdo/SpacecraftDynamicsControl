%% Run Tasks
% Runs and validates tasks

% Initial Orbit Frame Orientation Angles
PARAMS.w_b_n_lmo = [0; 0; 0.000884797]; % rad/s
PARAMS.w_b_n_gmo = [0; 0; 0.0000709003]; % rad/s
PARAMS.r_lmo = 3396.19 + 400; % km
PARAMS.r_gmo = 20424.2; % km
% (3-1-3) Euler Angles 
PARAMS.eu_lmo_init = [20;30;60];
PARAMS.eu_gmo_init = [0; 0; 250];





%% Task 1: Orbit Simulation

[n_pos_lmo, n_vel_lmo] = simulate_orbit(PARAMS.r_lmo, PARAMS.eu_lmo_init, PARAMS.w_b_n_lmo, 2000, 1);

% Plotting
figure;
plot3(n_pos_lmo(1,:), n_pos_lmo(2,:), n_pos_lmo(3,:), 'o')
xlabel('n_1 (km)');
ylabel('n_2 (km)');
zlabel('n_3 (km)');
xl = xlim();
yl = ylim();
zl = zlim();
hold on;
line([0,2000], [0,0], [0,0], 'LineWidth', 3, 'Color', 'k');
line([0,0], [0,2000], [0,0], 'LineWidth', 3, 'Color', 'k');
line([0,0], [0,0], [0,2000], 'LineWidth', 3, 'Color', 'k');
title('Orbit Simulations LMO and GMO')

%%
[n_pos_gmo, n_vel_gmo] = simulate_orbit(PARAMS.r_gmo, PARAMS.eu_gmo_init, PARAMS.w_b_n_gmo, 1150, 1);

% Plotting
hold on;
plot3(n_pos_gmo(1,:), n_pos_gmo(2,:), n_pos_gmo(3,:), 'o')

figure;
plot(n_pos_gmo(1,:));
hold on;
plot(n_pos_gmo(2,:));
hold on;
plot(n_pos_gmo(3,:));




