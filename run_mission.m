function [tout, yout, tlm] = run_mission(tend, dt, mode)
% Simulate a mission using a runge-kutta 4 integrator

% Setup initial conditions
o_b_n = [0.3; -0.4; 0.5]; % S/C Initial attitude
b_w_b_n = [1.00; 1.75; -2.20]; % S/C Iniital body angular velocity
x0 = [o_b_n; b_w_b_n*pi/180];

t = 0:dt:tend;
yout = zeros(length(x0), length(t));
yout(:, 1)= x0; % Setup Initial Conditions

% Setup output telemtetry
tlm = table();

% Start mission simulation
for i = 1:(length(t)-1)
    % Check what mode we are in
    switch mode
        case 'SUN-POINTING'
            % Get Sun-Pointing Reference Frame
            % From Inertial to Reference
            dcm_r_n = getRsN(); % From Inertial to Sun-Pointing
            % Get desired angular velocity
            n_ws_r_n = [0; 0; 0]; % rad/s
            
    end
    % Save setpoint variables to telemetry
    
    % Calculate Control Error
    [o_b_rs_n, b_w_b_rs_n] = calcAttErr(yout(1:3,i), yout(4:6, i), dcm_r_n, n_ws_r_n);
    % Save control error to telemetry
    
    
    % Calculate output
    
    
    % Propogate dynamics
    
end



end