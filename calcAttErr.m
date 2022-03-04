function [o_b_r, b_w_b_r] = calcAttErr(o_b_n, b_w_b_n, RN, n_w_r_n)
% Calculate Attitude and Angular velocity tracking error of current body
% frame B relative to reference frame R

o_b_r = o_b_n - dcm2mrp(RN);

BN = mrp2dcm(o_b_n);

b_w_b_r = b_w_b_n - BN' * n_w_r_n;




end