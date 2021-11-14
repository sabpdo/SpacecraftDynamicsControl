function [n_pos,n_vel] = simulate_orbit(radius, initial_angles, w_b_n, t, dt, gmo)

% Initialize variables
b_pos = [radius; 0; 0];
n_pos = [];
n_vel = [];
eu_angles = [];
eu_angles = [eu_angles initial_angles];

i = 1;
for i_t = 0:dt:t
    % Setup DCM
    dcm_n_b = get313DCM(eu_angles(:,i));
    
    % Update position vector
    n_pos_val = inv(dcm_n_b) * b_pos;
    n_pos = [n_pos n_pos_val];
    
    % Update velocity vector
    if (gmo == 1)
        w_n_b = (dcm_n_b) * w_b_n;
        n_vel_val = cross(w_n_b, b_pos);
        n_vel = [n_vel n_vel_val];
    else
        b_vel_val = cross(w_b_n, b_pos);
        n_vel_val = inv(dcm_n_b) * b_vel_val;
        n_vel = [n_vel n_vel_val];
    end
    
    % Get new euler rates (From body to inertial)
    if (gmo == 1)
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

