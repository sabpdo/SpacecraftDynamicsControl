function [n_pos,n_vel, dcm_n_b_t] = simulate_orbit(radius, initial_angles, w_b_n, t, dt, gmo)

% Initialize variables
b_pos = [radius; 0; 0];
n_pos = [];
n_vel = [];
eu_angles = [];
eu_angles = [eu_angles initial_angles];
dcm_n_b_t = []; % struct to hold all of HN(t)

i = 1;
for i_t = 0:dt:t
    % Setup DCM
    dcm_n_b = get313DCM(eu_angles(:,i));
    dcm_n_b_t = [dcm_n_b_t dcm_n_b];
    
    % Update position vector
    n_pos_val = inv(dcm_n_b) * b_pos;
    n_pos = [n_pos n_pos_val];
    
    % Update velocity vector
    b_vel_val = cross(w_b_n, b_pos);
    n_vel_val = inv(dcm_n_b) * b_vel_val;
    n_vel = [n_vel n_vel_val];

    
    % Get new euler rates (From body to inertial)
    if (gmo == 1)
        dcm_rate_n_b= get313DiffEq(eu_angles(:,i));
        w_n_b = inv(dcm_n_b) * w_b_n;
        new_angles_rad = eu_angles(:,i)*(pi/180) + w_n_b * dt;
    else
        dcm_rate_n_b= get313DiffEq(eu_angles(:,i));
        dcm_rate_b_n = inv(dcm_rate_n_b);
        eul_rates = dcm_rate_b_n * w_b_n; % from body to inertial rates
        % Integrate eul_rates and update eu_angles and interator (Euler Method)
        new_angles_rad = eu_angles(:,i)*(pi/180) + eul_rates * dt;
    end
    eu_angles = [eu_angles new_angles_rad*(180/pi)];
    i = i + 1;
        
end



end

