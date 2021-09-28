% ==========================
% ex03_benchmark1
% ==========================
%
% This script reproduces the first benchmark found in the paper "Fast
% and Robust Inverse Kinematics using Halley's method", by Steffan Lloyd,
% Rishad Irani, and Mojtaba Ahmadi.
%
% Performs a benchmarking comparison of the performance and
% reliability of the QuIK/NR/BFGS algorithms at different perturbation
% amounts.
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

%% Benchmark 1: NR/QuIK comparison
% Benchmark code is already packaged into the function
% benchmark.benchmark1.
%
% The test parameters are stored in the function benchmark.testParams. By
% default, this first benchmark only runs on the first 7 test parameters,
% which correspond to NR, QuIK, DNR, DQuIK, and BFGS (as in the paper).
% This can be customized by changing the "paramMask" name-value pair.
%
% The manipulator used is the KUKA KR6 manipulator, stored in
% robots.KUKA_KR6.
%
% The benchmark below should take about 10 seconds to run, as is (depending
% on the machine. You can try increasing the number of samples or rest-time
% to improve the consistency of the results.

% Run benchmark
[path1, data1] = benchmark.benchmark1( N = 1e4, ... % Number of samples
                                       restTime = 0, ... % Pause between samples to cool down CPU
                                       exitTol = 1e-8 ); % Which tests to run
  
%% Plot Benchmark 1
% This code is a little less clear, but for conciseness we leave it as is. 
% All it does is plot out 3 plots for mean iterations, time and reliability
% for the data collection above.
%
% Note that for the reliability plot, if there are no errors, the plot
% won't show these datapoints since 0 does not appear on a log-plot.

% Extract iter, time and reliability
t = mean( utils.structfield( data1.stats, 't'), 3);
meanIter = mean( utils.structfield( data1.stats, 'meanIter'), 3);
rel = mean( utils.structfield( data1.stats, 'reliability'), 3);

tiledlayout(3,1);

% Mean iter plot
nexttile;
hold on;
ax = gca; ax.XScale = 'log';
for i = 1:numel(data1.params)
    semilogx( data1.errorSizes, meanIter(:, i), strcat(data1.params{i}.linetype, data1.params{i}.marker), ...
        'color', data1.params{i}.color);
end
ylabel('Mean iterations to convergence');
xlabel('Perturbation amount');
grid on;
l = legend(cellfun(@(p) string(p.name), data1.params), 'location', 'northoutside', 'interpreter', 'latex');

% Time plot
nexttile;
hold on;
ax = gca;
ax.XScale = 'log';
for i = 1:numel(data1.params)
    semilogx( data1.errorSizes, t(:, i), strcat(data1.params{i}.linetype, data1.params{i}.marker), ...
        'color', data1.params{i}.color);
end
grid on;
ylabel('Mean evaluation time');
xlabel('Perturbation amount');

% Reliability plot
nexttile;
hold on;
ax = gca; ax.XScale = 'log'; ax.YScale = 'log';
for i = 1:numel(data1.params)
    loglog( data1.errorSizes, (1-rel(:, i))*100, strcat(data1.params{i}.linetype, data1.params{i}.marker), ...
        'color', data1.params{i}.color);
end
grid on;
ylabel('Non-convergence rate [%]');
xlabel('Perturbation amount');
