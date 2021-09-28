function [filepath, data] = benchmark3(robots, opt)

    arguments
        robots
        opt.N (1,1) double = 1e4 % Number of samples
        opt.N2 (1,1) double = 1 % Number of sample samples
        opt.RestTime (1,1) double = 1 % Time between benchmarks to allow cpu to "cool down"
        opt.iterMax (1,1) double = 200 % Convergence tolerance for time tests
        opt.exitTol (1,1) double = 1e-12 % Convergence tolerance for time tests
        opt.isConvergedTol (1,1) double = 1e-5 % Convergence tolerance for reliability tests
        opt.paramMask (:, 1) double = [] % Mask
    end
    
    % Get test parameters
    params = benchmark.testParams(iterMax = opt.iterMax, ...
                                  exitTol = opt.exitTol);
    if ~isempty( opt.paramMask )
        params = params(opt.paramMask);
    end
    nTests = numel(params);
    nRobots = numel(robots);
    
    %% Time collection
    utils.progress('Processing (time collection)...');

    % Time and stats variable
    stats = cell(nRobots, nTests, opt.N2);
    
    % Start timer
    t2 = tic;

    % Make test repeatable
    rng(10);
    
    % Loop through "number of samples"
    for k = 1:opt.N2
        for i = 1:nRobots
            
            r = robots(i);

            % Generate samples
            [Q, Twt] = benchmark.randomPose(r, opt.N, repeatable=false);


            Q0 = repmat( r.nominalPose, [1, opt.N] );

            % Loop through tests
            for j = 1:nTests
                utils.progress( ( (k-1)*nTests*nRobots + (i-1)*nTests + j)/nTests/opt.N2/nRobots );

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
    saveDir = fullfile( utils.QuIK_root, '+benchmark', 'save_data', 'time', subDir );
    if ~exist( saveDir, 'dir' )
        mkdir( saveDir );
    end
    time = clock;
    filename = sprintf('bench2_%s_%s_%d-%d-%d.mat', ...
        computer, date, time(4), time(5), round(time(6)));
    filepath = fullfile( saveDir, filename);
    
    data.stats = stats;
    data.opt = opt;
    data.params = params;
    data.robots = robots;
    
    save( filepath, ...
          'data');
      
    % Output
    fprintf("Done. Took %.1f minutes total.\n", toc(t2)/60);  
    fprintf("Data saved to %s\n\n", filepath );

end