function [filepath, data] = benchmark1(opt)
    % Benchmark1. Performs a benchmarking comparison of the performance and
    % reliability of the QuIK/NR/BFGS algorithms at different perturbation
    % amounts.
    %
    % Data is saved in matlab/+benchmark/save_data

    arguments
        opt.r = robots.KUKA_KR6;
        opt.N (1,1) double = 1e4 % Number of samples
        opt.N2 (1,1) double = 1 % How many repetitions to run
        opt.RestTime (1,1) double = 1 % Time between benchmarks to allow cpu to "cool down"
        opt.errorRange (1,2) double = [-2 0] % Exponents of upper and lower bounds to test 
        opt.errorStep (1,1) double = 0.25 % Increments of exponents between bounds
        opt.exitTol (1,1) double = 1e-12 % Convergence tolerance for time tests
        opt.isConvergedTol (1,1) double = 1e-5 % Convergence tolerance for reliability tests
        opt.paramMask (:, 1) double = 1:7 % Which tests to run on
    end
    
    % Get test parameters
    params = benchmark.testParams(exitTol = opt.exitTol);
    if ~isempty( opt.paramMask )
        params = params(opt.paramMask);
    end
    nTests = numel(params);
    
    % Break out robot variable
    r = opt.r;
    
    % Error sizes
    errorSizes = 10.^(opt.errorRange(1):opt.errorStep:opt.errorRange(2));
    nErrors = numel(errorSizes);
    
    %% Time collection
    utils.progress('Processing (time collection)...');

    % Time and stats variable
    stats = cell(nErrors, nTests, opt.N2);

    % Generate samples
    [Q, Twt] = benchmark.randomPose(r, opt.N, repeatable=true);
    
    % Start timer
    t2 = tic;
    
    % Make test repeatable
    rng(10);

    % Loop through "number of samples"
    for k = 1:opt.N2
        for i = 1:nErrors
            % Perturb poses
            Q0 = benchmark.perturbPose( Q, errorSizes(i), repeatable=false );

            % Loop through tests
            for j = 1:nTests
                utils.progress( ( (k-1)*nTests*nErrors + (i-1)*nTests + j)/nTests/opt.N2/nErrors );

                % Pause to allow cpu to cool
                pause(opt.RestTime)
                
                % Benchmark
                stats{i,j,k} = benchmark.runIK(r, Q, Q0, Twt, params{j});      
            end
        end
    end
    
    % Convert stats to array
    stats = cell2mat(stats);
    
    % Save results
    subDir = sprintf('%s_%s', computer, date);
    saveDir = fullfile( utils.QuIK_root, 'matlab', 'save_data', 'time', subDir );
    if ~exist( saveDir, 'dir' )
        mkdir( saveDir );
    end
    time = clock;
    filename = sprintf('time_%s_%s_%d-%d-%d.mat', ...
        computer, date, time(4), time(5), round(time(6)));
    filepath = fullfile( saveDir, filename);
    
    data.stats = stats;
    data.opt = opt;
    data.params = params;
    data.r = r;
    data.errorSizes = errorSizes;
    
    save( filepath, ...
          'data');
      
    % Output
    fprintf("Done. Took %.1f minutes total.\n", toc(t2)/60);  
    fprintf("Data saved to %s\n\n", filepath );

end