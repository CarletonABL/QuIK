% ==========================
% ex05_benchmark3
% ==========================
%
% This script reproduces the third benchmark found in the paper "Fast
% and Robust Inverse Kinematics using Halley's method", by Steffan Lloyd,
% Rishad Irani, and Mojtaba Ahmadi.
%
% Performs a benchmarking comparison of the performance and
% reliability of the QuIK/NR/BFGS algorithms with other algorithms, such as
% KDL and Matlab
%
% Results may vary from machine to machine, compiler to compiler, and
% depending on the sample sizes used for benchmarking.

%% Part 1: Code compilation
% To compile the C++ code, you must have an eigen3 directory somewhere on
% your computer. You can just clone this from the eigen github. You must
% tell matlab where to find this directory in /matlab/config.m.
% Once this is done, you can run the code below which generates the
% necessary mex-files.
%
% This code only needs to be run once per machine.

% Compile C++ QuIK codebase
QuIK_cpp.make();

% Compile KDL codebase
KDL_IK.make();

% Compile matlab functions into real time code.
% Note, this take a little longer since matlab is also converting the
% matlab code into real-time C-code, prior to compiling.
% Matlab also doesn't allow code to be compile for a generic robot, so a
% separate function must be compile for each individual robot. These
% functions are stored in IK_matlab. All files are compiled with the
% command IK_matlab.make().
% 
% Matlab coder and matlab robotics toolbox are required for this step.
IK_matlab.make();

%% Benchmark 3: NR/QuIK comparison with other state of the art
% Benchmark code is already packaged into the function
% benchmark.benchmark3.
%
% The test parameters are stored in the function benchmark.testParams. By
% default, this function will run on all algorithms, but this can be
% modified by altering the "paramMask" name-value pair in
% benchmark.benchmark3.
%
% This benchmark is much slower than benchmarks 1 and 2 to run, since we
% run it on all algorithms, including the matlab algorithms which are much
% much slower per sample. With N = 1e3, this should take about 2 minute to
% run. You can try increasing the number of samples or rest-time
% to improve the consistency of the results.

% Make an array of robots
rs = [robots.KUKA_KR6();
      robots.KUKA_KR6_cal();
      robots.KUKA_iiwa7();
      robots.jaco();
      robots.atlas()];

% Run benchmark
[filepath, data1] = benchmark.benchmark3( rs, ...
    N = 1e3,... % Number of samples to run
    restTime = 0, ... % Pause between samples
    paramMask = [1 2 4 6 7:11], ... % Which tests to run
    exitTol = 1e-8, ... % Exit tolerance
    isConvergedTol = 1e-5); % Tolerance for determining convergence
  
%% Print out results
% This code is a little less clear, but for conciseness we leave it as is. 
% All this does is print out the results in a more human readable format

% Extract iter, time and reliability
t = mean( utils.structfield( data1.stats, 't'), 3);
rel = mean( utils.structfield( data1.stats, 'reliability'), 3);
errorRate = (1-rel)*100;

for i = 1:numel(data1.robots)
    r = data1.robots(i);
    fprintf("\n\nResults for %s:\n", r.name);
    
    results = table;
    results.Algorithm = cellfun( @(pi) string(pi.name), data1.params');
    results.Time = arrayfun(@(ti) sprintf("%.5g us",ti), t(i, :)');
    results.ErrorRate = arrayfun(@(ei) sprintf("%.5g%%",ei), errorRate(i, :)');
    
    
    results.Properties.VariableNames = ["Algorithm", "Time", "ErrorRate"];
    results.Properties.VariableDescriptions = ["", "[us]", "[%]"];
    disp(results);
end