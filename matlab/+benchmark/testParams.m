function params = testParams(opt)
    % Makes a cell array of all the tests to be run, and their plot colors
    
    arguments
        opt.iterMax = 200
        opt.exitTol = 1e-10
        opt.rcondLimit = 0
    end
    
    % Plot settings
    lineWidth = 1.4;
    markerLineWidth = 1.0;
    markerSize = 5;
    
    colors(:, :, 1) = [12, 56, 199;
                  58, 194, 207;
                  55, 145, 58;
                  58, 207, 172;
                  83, 207, 58]/255;
    colors(:, :, 2) = [199, 20, 50;
                  212, 70, 200;
                  247, 180, 54;
                  247, 112, 54;
                  228, 247, 54]/255;
                  

    params = {};
    params{1} = struct('name', 'QuIK', ...
                       'opt', QuIK.options('iterMax', opt.iterMax, ...
                                           'exitTol', opt.exitTol, ...
                                           'algorithm', 0, ...
                                           'maxLinearErrorStep', 0.34, ...
                                           'maxAngularErrorStep', 1.00), ...
                       'color', colors(1, :, 1), ...
                       'marker', '^', ...
                       'linetype', '-', ...
                       'linewidth', lineWidth*2, ...
                       'markerLineWidth', markerLineWidth*1.5, ...
                       'markerSize', markerSize, ...
                       'entryFcnId', 1, ...
                       'numDerate', 1);

    params{2} = struct('name', 'NR', ...
                       'opt', QuIK.options( 'iterMax', opt.iterMax, ...
                                            'exitTol', opt.exitTol, ...
                                            'algorithm', 1, ...
                                            'maxLinearErrorStep', 0.14, ...
                                            'maxAngularErrorStep', 0.84), ...
                       'color', colors(1, :, 2), ...
                       'marker', 'o', ...
                       'linetype', '-.', ...
                       'linewidth', lineWidth*2, ...
                       'markerLineWidth', markerLineWidth*1.5, ...
                       'markerSize', markerSize, ...
                       'entryFcnId', 1, ...
                       'numDerate', 1);

    % Damped newton
    markers = {'s', 'd', 'v', '<', '>', 'p', '*', '+', 'x', 'd', '|', '_', 'h'};
    c = numel(params);
    names = ["DQuIK", "DNR"];
    lineTypes = {'-', '-.'};
    
    for alg = 0:1
        colCnt = 1;
        for i =[-5 -7]
            c = c+1;
            colCnt = colCnt + 1;
            params{c} = struct('name', sprintf('%s, $\\lambda^2 = 10^{%d}$', names(alg+1), i), ...
                               'shortname', names(alg+1), ...
                               'opt', QuIK.options( 'iterMax', opt.iterMax, ...
                                                    'exitTol', opt.exitTol, ...
                                                    'algorithm', alg, ...
                                                    'lambda', 10^i, ...
                                                    'maxLinearErrorStep', params{alg+1}.opt.maxLinearErrorStep, ...
                                                    'maxAngularErrorStep', params{alg+1}.opt.maxAngularErrorStep), ...
                               'color', colors(colCnt, :, alg+1), ...
                               'marker', markers{c-2}, ...
                               'linetype', lineTypes{alg+1}, ...
                               'linewidth', lineWidth, ...
                               'markerLineWidth', markerLineWidth, ...
                               'markerSize', markerSize, ...
                               'entryFcnId', 1, ...
                               'numDerate', 1);
        end
    end
        
    % BFGS, custom
    c = c+1;
    colCnt = colCnt + 1;
    params{c} = struct('name', 'BFGS', ...
                           'opt', QuIK.options( 'iterMax', opt.iterMax*2, ...
                                               'exitTol', opt.exitTol, ...
                                               'algorithm', 2, ... % BFGS
                                               'maxLinearErrorStep', inf, ...
                                               'maxAngularErrorStep', inf), ...
                            'color', [128, 34, 242]/255, ...
                            'marker', markers{7}, ...
                            'linetype', '--', ...
                            'linewidth', lineWidth, ...
                            'markerLineWidth', markerLineWidth, ...
                            'markerSize', markerSize, ...
                            'entryFcnId', 1, ...
                            'numDerate', 1 );
                        
    % KDL LMA
    c = c+1;
    params{c} = struct('name', 'KDL-LMA', ...
                       'opt', QuIK.options( 'iterMax', opt.iterMax, ...
                                           'exitTol', opt.exitTol, ...
                                           'algorithm', 3), ...
                        'color', [0, 217, 177]/255, ...
                        'marker', markers{5}, ...
                        'linetype', ':', ...
                       'linewidth', lineWidth, ...
                        'markerLineWidth', markerLineWidth, ...
                        'markerSize', markerSize, ...
                        'entryFcnId', 3, ...
                        'numDerate', 100 );

    % KDL NR
    c = c+1;
    params{c} = struct('name', 'KDL-NR', ...
                       'opt', QuIK.options( 'iterMax', opt.iterMax, ...
                                           'exitTol', opt.exitTol, ...
                                           'algorithm', 1), ...
                        'color', utils.plotColors(5,8), ...
                        'marker', markers{5}, ...
                        'linetype', ':', ...
                            'linewidth', lineWidth, ...
                        'markerLineWidth', markerLineWidth, ...
                        'markerSize', markerSize, ...
                        'entryFcnId', 3, ...
                        'numDerate', 100 );
                        
    % Matlab
    names = {'ML-BFGS', 'ML-LMA'};
    for alg = 1:2
        c = c+1;
        params{c} = struct('name', names{alg}, ...
                           'opt', QuIK.options( 'iterMax', opt.iterMax, ...
                                               'exitTol', opt.exitTol, ...
                                               'algorithm', alg, ...
                                               'maxLinearErrorStep', params{2}.opt.maxLinearErrorStep, ...
                                               'maxAngularErrorStep', inf, ...
                                               'lambda', 0), ...
                            'color', utils.plotColors(6+alg,8), ...
                            'marker', markers{5+alg}, ...
                            'linetype', ':', ...
                            'linewidth', lineWidth, ...
                            'markerLineWidth', markerLineWidth, ...
                            'markerSize', markerSize, ...
                            'entryFcnId', 2, ...
                            'numDerate', 100 );
    end
end