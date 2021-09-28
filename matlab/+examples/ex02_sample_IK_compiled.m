% ==========================
% ex02_sample_IK_compiled
% ==========================
%
% This script demonstrates the use of the C++ package in matlab.
% Note, you must have a valid C++ compiler installed on your computer, and
% configured to run with matlab. Check the help for mex -setup c++ for more
% details.

%% Part 1: Code compilation
% To compile the C++ code, you must have an eigen3 directory somewhere on
% your computer. You can just clone this from the eigen github. You must
% tell matlab where to find this directory in /matlab/config.m.
% Once this is done, you can run the code below which generates the
% necessary mex-files.
%
% This code only needs to be run once per machine.

QuIK_cpp.make();

%% Part 2: Setup Test

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
N = 1e3;
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


%% Part 3: Benchmark
% This might take a few seconds, depending on the machine.

% Call inverse kinematics to find the original angles (matlab code)
t_matlab = timeit( @() QuIK.IK( r, Twt, Q0, opt ) )/N;

% Call inverse kinematics to find the original angles (c++ code)
t_cpp = timeit( @() QuIK_cpp.IK( r, Twt, Q0, opt ) )/N;

fprintf("QuIK matlab library took is %.1f us per sample.\n", t_matlab*1e6);
fprintf("C++ took is %.1f us per sample.\n", t_cpp*1e6);
fprintf("C++ is %.1f times faster.\n", t_matlab/t_cpp);

