function [stats, sol] = runIK(r, Q, Q0, Twt, param, opt)
    % Interface function, gives a unified interface for the different MEX
    % calling interfaces
    %
    % [stats, sol] = benchmark.runIK(r, Q, Q0, Twt, param, opts)
    
    arguments
        r
        Q
        Q0
        Twt
        param
        opt.isConvergedTol = 1e-5 % Tolerance to determine if a sample is converged
    end
    
    % Number of samples
    N = size(Q0,2);
    
    % Derate number of samples given to algorithm
    if param.numDerate > 1 && false
        N = round(N/param.numDerate);
        Q = Q(:, 1:N);
        Q0 = Q0(:, 1:N);
        Twt = Twt(:, :, 1:N);
    end
        
    % Time it
    t1 = tic;
    switch param.entryFcnId
        case 1
            % C++ custom library
            [sol.Q_star, ~, sol.iter, sol.breakReason] = QuIK_cpp.IK(r, Twt, Q0, param.opt);
        case 2
            % Matlab. Because of restrictions, we needed to compile a
            % separate file for each robot...
            switch r.id
                case 'KUKA_KR6'
                    [sol.Q_star, ~, sol.iter, sol.breakReason] = IK_matlab.IK_KUKA_KR6_mex(Twt, Q0, param.opt);
                case 'KUKA_KR6_cal'
                    [sol.Q_star, ~, sol.iter, sol.breakReason] = IK_matlab.IK_KUKA_KR6_cal_mex(Twt, Q0, param.opt);
                case 'KUKA_iiwa7'
                    [sol.Q_star, ~, sol.iter, sol.breakReason] = IK_matlab.IK_KUKA_iiwa7_mex(Twt, Q0, param.opt);
                case 'jaco'
                    [sol.Q_star, ~, sol.iter, sol.breakReason] = IK_matlab.IK_jaco_mex(Twt, Q0, param.opt);
                case 'atlas'
                    [sol.Q_star, ~, sol.iter, sol.breakReason] = IK_matlab.IK_atlas_mex(Twt, Q0, param.opt);
                otherwise
                    error('Invalid robot given');
            end
        case 3
            % KDL library
            [sol.Q_star, ~, sol.breakReason] = KDL_IK.IK(r.DHx, r.Tbase, r.Ttool, utils.vertStack(Twt), Q0, param.opt);
            sol.iter = int32(zeros(N,1));
    end            
    t = toc(t1);
    stats.t = t/N*1e6;
    
    % Get error
    [e_star, sol.c] = QuIK.IKeval(struct('DHx', r.DHx, 'Tbase', r.Tbase, 'Ttool', r.Ttool), ...
                                  Q, sol.Q_star );
    sol.e_star = vecnorm( e_star, 2, 1 );
    
    % Figure out if converged
    sol.converged = sol.e_star(:) <= opt.isConvergedTol .* sol.c(:);
    stats.nConverged = nnz(sol.converged);
    stats.reliability = stats.nConverged/N;
    
    sol.converged_noCond = sol.e_star(:) <= opt.isConvergedTol;
    stats.nConverged_noCond = nnz(sol.converged_noCond);
    stats.reliability_noCond = stats.nConverged_noCond/N;

    
    % Mean iterations stats
    stats.meanIter = mean(double(sol.iter));
end