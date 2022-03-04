function dcm = mrp2dcm(mrp)
% Convert MRP to DCM

% Normalize
mrp_n = norm(mrp);

% Skew-Sym Matrix
mrp_t = [0 -mrp_n mrp_n; mrp_n 0 -mrp_n; -mrp_n mrp_n 0];


dcm = eye(3) + ( 8*mrp_t^2 - 4*(1-mrp_n^2)*mrp_t ) / (1 - mrp_n^2)^2;



end