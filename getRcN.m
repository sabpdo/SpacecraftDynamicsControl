function [RcN] = getRcN(r_lmo, r_gmo)
% Get GMO-Pointing Reference Frame Orientation
%
% Position LMO S/C is away from GMO S/C:
%   dr = r_lmo - r_gmo
%
% Rc = {-dr/|dr|, cross(dr, n3 / |cross(dr, nr)|, cross(r1,r2)}
%       - -r1 points to GMO S/C
%       - r2 fully defines a 3-D reference frame
%       - r3 fully defines a 3-D reference frame




end


