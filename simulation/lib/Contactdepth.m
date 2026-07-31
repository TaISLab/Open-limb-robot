function a = contactDepth(theta0, l3, l4, rw)
% CONTACTDEPTH  Averaged central contact depth from swing-link deflections.
%   a = contactDepth(theta0, l3, l4, rw) implements Eq. (1) of the paper:
%   a(theta0) = l4 + (l3/2)*(sin(theta0f) + sin(theta0b)) + rw
%
%   theta0 : [theta0_front, theta0_back] swing-link deflections [rad]
%   l3     : swing-link length [m]
%   l4     : swing-link pivot y-offset [m]
%   rw     : wheel radius [m]
a = l4 + (l3/2)*(sin(theta0(1)) + sin(theta0(2))) + rw;
end
