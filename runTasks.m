%% Run Tasks
% Runs and validates tasks

% Initial Orbit Frame Orientation Angles
PARAMS.w_b_n_lmo = [0; 0; 0.000884797]; % rad/s
PARAMS.r_lmo = 3396.19 + 400; % km

% (3-1-3) Euler Angles 
PARAMS.eu_lmo_init = [20;30;60];





%% Task 1: Orbit Simulation

[n_pos, n_vel] = simulate_orbit(PARAMS.r_lmo, PARAMS.eu_lmo_init, PARAMS.w_b_n_lmo, 450, 1);

% Plotting
plot3(n_pos(1,:), n_pos(2,:), n_pos(3,:), 'o')
xlabel('n_1 (km)');
ylabel('n_2 (km)');
zlabel('n_3 (km)');
%plot3(n_vel(1,:), n_vel(2,:), n_vel(3,:), 'o')
