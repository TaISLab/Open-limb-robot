function theta_a = solveArmAngle(R, l0, l1, l3, l4, rw)
% SOLVEARMANGLE  Numerically inverts Eq. (3) to find the arm angle
%   theta_a that closes the grasp on a cylinder of known radius R,
%   assuming unloaded swing links (theta0 = [0 0]).
theta0 = [0 0];
f = @(th) radiusEstimator(th, theta0, l0, l1, l3, l4, rw) - R;
theta_a = fzero(f, deg2rad(70));
end
