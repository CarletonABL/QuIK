% =========================
% ex01_sample_IK
% =========================
%
% This script demonstrates using the matlab library to solve inverse
% kinematics on a KUKA KR6 robot.
% Written by Steffan Lloyd on Sep 27, 2021

% Load in robot.
r = robots.KUKA_KR6;

% Inverse kinematics options
% algorithm = 0 indicates that the QuIK algorithm should be used. 1 =
% Newton-Raphson, and 2 is for BFGS.
%
% Many other options exist, for details type "help QuIK.options"
opt = QuIK.options(algorithm = 0);

% Generate some random poses, stored in a DOFxN matrix.
% where N is the number of poses.
N = 5;
Q = rand(r.DOF, N);

% Do forward kinematics
% QuIK.FK function returns a 4x4x(DOF+1) page matrix, with each "page"
% representing the transform to the ith frame.
% We only care about the last frame
Twt = zeros(4,4,N);
for i = 1:N
    Twt_i = QuIK.FK(r, Q(:, i));
    Twt(:, :, i) = Twt_i(:, :, end);
end

% Perturb the poses slightly
% This code perturbs them with random numbers in the range -0.1 to 0.1
Q0 = Q + 0.1*2*(rand(r.DOF, N)-.5);

% Call inverse kinematics to find the original angles
[Q_star, ec] = QuIK.IK( r, Twt, Q0, opt );

% Output results
fprintf("\n\nTrue joint angles\n");
disp(Q);
fprintf("\n\nSolved joint angles\n");
disp(Q_star);
fprintf("\n\nError (normed)\n");
disp( vecnorm(horzcat(ec.e), 2, 1) );
fprintf("\n\nBreak reason (1 == converged)\n");
disp([ec.breakReason]);
fprintf("\n\nNumber of iterations\n");
disp([ec.iter]);