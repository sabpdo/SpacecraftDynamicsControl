function [dcm_rn_n,  = getRnN(t, initial_angles, w_b_n)
% Get Nadir-Pointing Reference Frame to Inertial Frame DCM
% Gives RnN at given time t
% H = {ir, itheta, ih}
% Rn = {-ir, itheta, cross(-ir, itheta)
%     - r1 points to planet
%     - r2 points to vel (RAM) direction

% Nadir-Pointing Reference Frame to Orbit frame
% RnH = {0, 180, 0} euler angle 313 sequence
RnH = [-1 0 0; 0 1 0; 0 0 -1];

dt = 1;
i = 1;


% Propogate orbit and get euler angles at time t
dcm_rate_n_b = get313DiffEq(initial_angles);
dcm_rate_b_n = inv(dcm_rate_n_b);
eul_rates = dcm_rate_b_n * w_b_n; % from body to inertial rates
% Integrate eul_rates and update eu_angles via Euler's Method
eul_angles = initial_angles * (pi/180) + eul_rates * t;


% Get DCM from inertial to orbit frame
% [NH]
dcm_n_b = get313DCM(eu_angles);
% Transpose DCM [HN]
dcm_b_n = dcm_n_b';

% Compute [RnN] = [RnH] * [HN]
dcm_rn_n = RnH * dcm_b_n;



end