% ==========================
% ex04_benchmark2
% ==========================
%
% This script reproduces the first benchmark found in the paper "Fast
% and Robust Inverse Kinematics for Serial Robots using Halley's method", 
% by Steffan Lloyd, Rishad Irani, and Mojtaba Ahmadi.
% DOI: 10.1109/TRO.2022.3162954
% 
% Performs a comparison of the singularity robustness of the algorithms.
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

%% Benchmark 2: NR/QuIK singularity robustness
% Benchmark code is already packaged into the function benchmark.benchmark2
%
% The test parameters are stored in the function benchmark.testParams. By
% default, this first benchmark only runs on the first 7 test parameters,
% which correspond to NR, QuIK, DNR, DQuIK, and BFGS (as in the paper).
% This can be customized by changing the "paramMask" name-value pair.
%
% The benchmark is run in two parts. The first part does an automatic
% search for near-singular poses (with certain condition numbers) and
% stores them in a savefile.
%
% Generate singular poses
[path1, data1] = benchmark.benchmark2( N = 1e3 ); % Number of samples to make

%% Run benchmarks on poses
% The second part runs the benchmark on these poses.

[path2, data2] = benchmark.benchmark2(  exitTol = 1e-12, ...
                                        partialSave = path1);

%% Plot Benchmark 2
% This code is a little less clear, but for conciseness we leave it as is. 
% Note that if there are no errors, the plot won't show these datapoints
% since 0 does not appear on a log-plot.

% Create tiled layout
tiledlayout(2,1);

% Create a cell array of data to plot
% data.rel1 contains the singular target values, data.rel2 contains the
% reliability for singular source
rel = {data2.rel1, data2.rel2};

% Create one plot for each
for k = 1:2
    nexttile;
    hold on;
    ax = gca; ax.YScale = 'log';
    for i = 1:numel(data2.params)
        semilogy( log10(data2.condNums), (1-rel{k}(i, :))*100, strcat(data2.params{i}.linetype, data2.params{i}.marker), ...
            'color', data2.params{i}.color);
    end
    grid on;
    ylabel('Non-convergence rate [%]');
    xlabel('Perturbation amount');
    
    % Title each
    titles = ["Singular Target Configuration";
              "Singular Starting Configuration"];
    title(titles(k));
    
    xlabel("Condition number ||\bf{J}(\bf{q})||");
end
