function x = lsolve(A, b, opt)
    % Linear system solver for the IK algorithms
    % lsolve(A, b, opt)
    
    coder.inline('always');
    M = size(A, 1);
    
    % Algorithm to use depends on a few factors
    % If lambda > 0, then we use the covariance matrix method (maybe exploiting
    % the PD property of the resulting matrix).
    Astar = A*A';

    if opt.lambda2 > 0
        for i = 1:M
            Astar(i,i) = Astar(i,i) + opt.lambda2;
        end
    end

    % If M is larger, it is worthwhile to exploit the positive
    % definiteness of the matrix. Otherwise, it just adds overhead.
    % The 8 DOF threshold is heuristically chosen based on limited
    % testing.
    if M > 8
        linOptsPD = coder.const( struct('SYM', true, 'POSDEF', true) );
        x = A' * linsolve(Astar, b, linOptsPD);
    else
        x = A' * linsolve(Astar, b);
    end
    
end