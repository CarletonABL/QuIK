function [Qstar, e, iter, breakReason] = IK(robot, Twt, Q0, opt)
    % Evaluates the inverse kinematics of a chain in a loop
    
    % Break out some vars
    N = size(Twt,3);
    DOF = size(Q0, 1);
    
    % Preallocate
    Qstar = coder.nullcopy(zeros(DOF, N));
    solInfo = repmat( struct( 'Iterations', 0, ...
                              'NumRandomRestarts', 0, ...
                              'PoseErrorNorm', 0, ...
                              'ExitFlag', 0, ...
                              'Status', '' ), N, 1);
    coder.varsize('solInfo.Status', [1 100], [0 1]);
    
    % Initialize algorithm
    if opt.algorithm == 2
        % BFGS
        % Solver options
        solverOpts_bfgs = struct('MaxIterations', double(opt.iterMax), ...
                                'StepTolerance', opt.minStepSize, ...
                                'SolutionTolerance', opt.exitTol, ...
                                'EnforceJointLimits', false, ...
                                'AllowRandomRestarts', false);
        ik_bfgs = inverseKinematics('RigidBodyTree', robot, ...
                               'SolverAlgorithm', 'BFGSGradientProjection', ...
                               'SolverParameters', solverOpts_bfgs);
                            
        for i = 1:N
            [Qstar(:, i), solInfo(i)] = ik_bfgs('tool', Twt(:, :, i), ones(6,1), Q0(:, i));
        end
        
    elseif opt.algorithm == 1
        % LMA
        % Solver options
        solverOpts_lma = struct('MaxIterations', double(opt.iterMax), ...
                                'StepTolerance', opt.minStepSize, ...
                                'SolutionTolerance', opt.exitTol, ...
                                'EnforceJointLimits', false, ...
                                'AllowRandomRestarts', false, ...
                                'DampingBias', max(2e-14,sqrt(opt.lambda2)), ...
                                'UseErrorDamping', sqrt(opt.lambda2) > 1e-14, ...
                                'ErrorChangeTolerance', opt.maxLinearErrorStep );
                        
        ik_lma = inverseKinematics('RigidBodyTree', robot, ...
                               'SolverAlgorithm', 'LevenbergMarquardt', ...
                               'SolverParameters', solverOpts_lma);
                           
        for i = 1:N
            [Qstar(:, i), solInfo(i)] = ik_lma('tool', Twt(:, :, i), ones(6,1), Q0(:, i));
        end
        
    else
        error("Invalid algorithm");
    end
    
    disp(solInfo)
        
    e = [solInfo.PoseErrorNorm];
    iter = int32([solInfo.Iterations]);
    breakReason = int32([solInfo.ExitFlag]);
    
end