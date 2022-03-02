function [dcm_rn_n, n_w_rn_n]  = getRnN(initial_angles, w_b_n, t, dt)
% Get Nadir-Pointing Reference Frame to Inertial Frame DCM
% Gives RnN at given time t
% H = {ir, itheta, ih}
% Rn = {-ir, itheta, cross(-ir, itheta)
%     - r1 points to planet
%     - r2 points to vel (RAM) direction

% Nadir-Pointing Reference Frame to Orbit frame
% RnH = {0, 180, 0} euler angle 313 sequence
RnH = [-1 0 0; 0 1 0; 0 0 -1];


% Propogate orbit and get the dcm at time t
[~, ~, dcm_n_b_t] = simulate_orbit(0, initial_angles, w_b_n, t, dt, 0);
dcm_n_b_t = dcm_n_b_t(:,end-2:end)'; % Just need to get the last one
dcm_b_n_t = dcm_n_b_t';


% Compute [RnN] = [RnH] * [HN]
dcm_rn_n = RnH * dcm_b_n_t;

% Return angular velocity vector n_w_rn_n
n_w_rn_n = dcm_n_b_t * w_b_n;


end