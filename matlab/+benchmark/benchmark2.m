function [filepath, data] = benchmark2(opt)
    % Performs the singularity benchmarking
    %
    % This is done in two steps. If run with default parameters, the script
    % will generate a set of N near-singular poses for use in part two.
    % These are saved in a savefile and returned by the script.
    %
    % In the second stage, the parameter partialSave must be set to the
    % path to the file in step 1. Then, the algorithms will actually be
    % tested on these poses.

    arguments
        opt.N (1,1) double = 1e4 % Number of samples
        opt.exitTol (1,1) double = 1e-12 % Convergence tolerance for time tests
        opt.paramMask (:, 1) double = 1:7 % Tests to run on
        opt.partialSave = [] % Savefile to partial data collection
        opt.perturbation = 0.1;
        opt.isConvergedTol = 1e-3;
    end
    
    % Read in robot
    r = robots.KUKA_KR6;
    
    % Make tests repeatable by resetting random number generator
    rng(11); 
        
    % Start timer
    t2 = tic;

    if isempty(opt.partialSave)
        %% STEP 1
        
        % Step one - generate an identically singular pose for the KUKA robot
        yw = .4;
        a3 = abs(r.DHx(3,1));
        a2 = abs(r.DHx(2,1));
        a1 = abs(r.DHx(1,1));
        d4 = abs(r.DHx(4,3));
        C = sqrt(a3^2 + d4^2);
        D = sqrt(a1^2 + yw^2);
        beta = acos( - (D^2 - C^2 - a2^2 )/(2*C*a2) );
        alpha = atan(d4/a3);
        q3 = 3/2*pi - beta - alpha - pi/2;
        gamma = asin( sin(beta) * C/D );
        theta = atan(a1/yw);
        q2 = gamma + theta - pi/2;
        q1 = 0;
        q_singular = [q1; -q2; q3; 0.1; 0.1; 0.1];

        % Condition numbers to generate at
        condNums = 10.^[2:1:15];
        nConds = numel(condNums);

        % Generate perturbation from singularity
        Qpert = (rand(r.DOF, opt.N) - .5) * 1;
        Qtarget = zeros(r.DOF, opt.N, nConds);

        % Run through generation process
        % This will take a while
        utils.progress('processing')
        for j = 1:nConds
            utils.progress(j/nConds);
            condNum = condNums(j);
            parfor i = 1:opt.N
                % Select out a random direction vector
                qpert_i = Qpert(:, i);

                % Define a function in one variable (sigma)
                % This function takes the singular pose, and moves away from it in
                % the direction of q_pert_i (random).
                % Eventually, the condition number will increase since this is an
                % isolated singularity.
                % The fzero algorithm finds the point where sigma is the correct
                % size to give the desired condition number
                % Then the result is stored in Qtarget.
                f = @(sigma) condNum - QuIK.poseCond(r, q_singular + sigma*qpert_i);
                sigma_i = fzero(f, 1/condNum);
                Qtarget(:, i, j) = q_singular + sigma_i * qpert_i;
            end
        end

        % Save results
        benchDir = fileparts(mfilename('fullpath'));
        subDir = sprintf('%s_%s', computer, date);
        saveDir = fullfile( benchDir, 'save_data', 'singular_poses', subDir );
        if ~exist( saveDir, 'dir' )
            mkdir( saveDir );
        end
        time = clock;
        filename = sprintf('singular_poses_%s_%s_%d-%d-%d.mat', ...
            computer, date, time(4), time(5), round(time(6)));
        filepath = fullfile( saveDir, filename);

        data.N = opt.N;
        data.condNums = condNums;
        data.r = r;
        data.Qtarget = Qtarget;
        data.q_singular = q_singular;

        save( filepath, ...
              'data');
          
        % Output
        fprintf("Done. Took %.1f minutes total.\n", toc(t2)/60);  
        fprintf("Data saved to %s\n\n", filepath );
    
    else 
        %% Step 2: Run the tests
        params = benchmark.testParams(exitTol = opt.exitTol);
        if ~isempty(opt.paramMask)
            params = params(opt.paramMask);
        end
        nTests = numel(params);

        % Load in data
        S = load(opt.partialSave, 'data');
        data1 = S.data;
        
        nConds = numel(data1.condNums);
        N = data1.N;
        
        % Initiate variables
        rel1 = zeros(nTests, nConds);
        err1 = zeros(nTests, nConds);
        rel2 = zeros(nTests, nConds);
        err2 = zeros(nTests, nConds);
        
        % Iterate through tests
        utils.progress( 'processing' );
        for k = 1:nTests
            param = params{k};

            for j = 1:nConds
                
                utils.progress(((k-1)*nConds + j)/nTests/nConds)

                % Target poses
                Qtarget_j = data1.Qtarget(:, 1:N, j);
                
                % Perturb result and get starting points 
                Qstart_j = benchmark.perturbPose( Qtarget_j, opt.perturbation, repeatable = true);

                % Forward kinematics
                Twt_target = RU.FK(r.DHx, r.Tbase, Qtarget_j, 7, r.Ttool);
                Twt_start = RU.FK(r.DHx, r.Tbase, Qstart_j, 7, r.Ttool);

                % Run Ik
                [stats1, sol1] = benchmark.runIK(r, Qtarget_j, Qstart_j, Twt_target, param, isConvergedTol = opt.isConvergedTol);
                [stats2, sol2] = benchmark.runIK(r, Qstart_j, Qtarget_j, Twt_start, param, isConvergedTol = opt.isConvergedTol);

                % Error
                err1(k, j) = median(log10(sol1.e_star), 'omitnan');
                err2(k, j) = median(log10(sol2.e_star), 'omitnan');

                % Get reliability
                rel1(k, j) = stats1.reliability_noCond;
                rel2(k, j) = stats2.reliability_noCond;

            end
        end

        % Save results
        subDir = sprintf('%s_%s', computer, date);
        saveDir = fullfile( utils.QuIK_root, '+benchmark', 'save_data', 'singular_bench', subDir );
        if ~exist( saveDir, 'dir' )
            mkdir( saveDir );
        end
        time = clock;
        filename = sprintf('singular_bench_%s_%s_%d-%d-%d.mat', ...
            computer, date, time(4), time(5), round(time(6)));
        filepath = fullfile( saveDir, filename);

        data.params = params;
        data.r = r;
        data.err1 = err1;
        data.rel1 = rel1;
        data.err2 = err2;
        data.rel2 = rel2;
        data.condNums = data1.condNums;
        data.N = N;
        data.opt = opt;

        save( filepath, ...
              'data');
          
        % Output
        fprintf("Done. Took %.1f minutes total.\n", toc(t2)/60);  
        fprintf("Data saved to %s\n\n", filepath );
    end
    
end