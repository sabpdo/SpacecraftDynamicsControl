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

[n_pos_lmo, n_vel_lmo, ~] = simulate_orbit(PARAMS.r_lmo, PARAMS.eu_lmo_init, PARAMS.w_b_n_lmo, 1150, 1, 0);
disp('rLMO and vLMO at 450s: ');
disp(n_pos_lmo(:,end))
disp(n_vel_lmo(:,end))
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

% Simulate GMO
[n_pos_gmo, n_vel_gmo, ~] = simulate_orbit(PARAMS.r_gmo, PARAMS.eu_gmo_init, PARAMS.w_b_n_gmo, 1150, 1, 1);
disp('rGMO and vGMO at 1150s: ');
disp(n_pos_gmo(:,end))
disp(n_vel_gmo(:,end))



% Plotting
hold on;
plot3(n_pos_gmo(1,:), n_pos_gmo(2,:), n_pos_gmo(3,:), 'o')

%% Task 2: Orbit Frame Orientation

% Find LMO DCM [HN] at 300s
[~, ~, dcm_n_b_t] = simulate_orbit(PARAMS.r_lmo, PARAMS.eu_lmo_init, PARAMS.w_b_n_lmo, 300, 1, 0);

HN_300s = dcm_n_b_t(:,end-2:end);
disp('LMO HN Matrix at 300 s: ');
disp(HN_300s)

%% Task 3: Sun Pointing Reference Frame Orientation

dcm_rs_n = getRsN();
disp("Sun Pointing Reference Frame Orientation DCM: ")
disp(dcm_rs_n)

% w_rs_n is 0, Rs frame moves constant with respect to the inertial frame

%% Task 4: Nadir-Pointing Reference Frame

% Return [RnN] and n_w_rn_n at t = 330s

[dcm_rn_n, n_w_rn_n] = getRnN(PARAMS.eu_lmo_init, PARAMS.w_b_n_lmo, 330, 1);




