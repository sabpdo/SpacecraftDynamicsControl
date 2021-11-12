function [n_pos,n_vel] = simulate_orbit(radius, initial_angles, w_b_n, t, dt)

% Initialize variables

% Get 3-1-3 DCM
dcm_b_n = get313DCM(initial_angles);

b_pos = [radius; 0; 0];
n_pos = [];
n_vel = [];
eu_angles = [];
eu_angles = [eu_angles initial_angles];

i = 1;
for i_t = 1:dt:t
    % Setup DCM
    dcm_b_n = get313DCM(eu_angles(:,i));
    
    % Update position vector
    n_pos_val = dcm_b_n * b_pos;
    n_pos = [n_pos n_pos_val];
    
    % Update velocity vector
    n_vel_val = cross(w_b_n, b_pos);
    n_vel = [n_vel n_vel_val];
    
    % Get new euler rates (From body to inertial)
    dcm_rate_n_b= get313DiffEq(eu_angles(:,i));
    dcm_rate_b_n = inv(dcm_rate_n_b);
    eul_rates = dcm_rate_b_n * w_b_n;
    
    % Integrate eul_rates and update eu_angles and interator (Euler Method)
    new_angles_rad = eu_angles(:,i)*(pi/180) + eul_rates * dt;
    eu_angles = [eu_angles new_angles_rad *(180/pi)];
    i = i + 1;
        
end



end

