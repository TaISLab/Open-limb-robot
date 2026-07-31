function gamma = lateralContactAngle(theta_a, theta0, l0, l1, l3, l4, rw)
% LATERALCONTACTANGLE  Lateral-contact angle gamma (Eq. 4 of the paper).
%   gamma = arcsin( P1x / (R_est + rw) )
%
%   See radiusEstimator.m for argument definitions.
R = radiusEstimator(theta_a, theta0, l0, l1, l3, l4, rw);
P1x = l0 + l1*cos(theta_a);
gamma = asin(P1x / (R + rw));
end
