function dcm_rn_n = getRnN(t)
% Get DCM from Rn to Inertial frame
% H = {ir, itheta, ih}
% Rn = {-ir, itheta, cross(-ir, itheta)
% RnH = {0, 180, 0} euler angle 313 sequence
RnH = [-1 0 0; 0 1 0; 0 0 -1];


end