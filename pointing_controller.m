function b_u = pointing_controller(o_b_r, b_w_b_r)

% Calculate control torques u (x, y, and z) 
% Controller to drive body frame B towards reference R
K = [1 1 1];
P = [1 1 1];
b_u = -K * o_b_r - P * b_w_b_r;


end