function R = radiusEstimator(theta_a, theta0, l0, l1, l3, l4, rw)
% RADIUSESTIMATOR  Closed-form proprioceptive cylinder-radius estimator.
%   R = radiusEstimator(theta_a, theta0, l0, l1, l3, l4, rw) implements
%   Eq. (3) of the paper:
%   R_est = (rw^2 - P1x^2 - (P1y+a)^2) / (2*(P1y+a-rw))
%
%   theta_a : arm angle [rad]
%   theta0  : [theta0_front, theta0_back] swing-link deflections [rad]
%   l0,l1   : arm pivot x-offset, arm length [m]
%   l3,l4   : swing-link length, pivot y-offset [m]
%   rw      : wheel radius [m]
a = contactDepth(theta0, l3, l4, rw);
P1x = l0 + l1*cos(theta_a);
P1y = -l1*sin(theta_a);
u = P1y + a;
R = (rw^2 - P1x^2 - u^2) / (2*(u - rw));
end
