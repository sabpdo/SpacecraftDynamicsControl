%% Run Tasks
% Runs and validates tasks

% Initial Orbit Frame Orientation Angles
PARAMS.w_b_n_lmo = [0; 0.000884797; 0]; % rad/s
PARAMS.r_lmo = 3396 + 400; % km

% (3-1-3) Euler Angles 
PARAMS.eu_lmo_init = [20;30;60];





%% Task 1: Orbit Simulation

[n_pos, n_vel] = simulate_orbit(PARAMS.r_lmo, PARAMS.eu_lmo_init, PARAMS.w_b_n_lmo, 450, 2);

% Plotting
plot3(n_pos(1,1), n_pos(2,1), n_pos(3,1), 'ro')

%plot3(n_vel(1,:), n_vel(2,:), n_vel(3,:), 'o')
