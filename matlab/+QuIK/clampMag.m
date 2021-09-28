function ei = clampMag(ei, opt)
    % ClampMag Saturates the magnitude of the error vector sent to the
    % algorithm.
    %
    % ei = clampMag(ei, opt)
    %
    % Implemented as described here: 
    %   [1] S. R. Buss, “Introduction to Inverse Kinematics
    %   with Jacobian Transpose, Pseudoinverse and Damped Least
    %   Squares methods,” 2009.
    % https://www.math.ucsd.edu/~sbuss/ResearchWeb/ikmethods/iksurvey.pdf
    
    % Don't do this for the BFGS algorithm.
    if opt.algorithm == 2
        return;
    end
        
    % Get squared error (avoid square root unless necessary)
    ei_lin_norm2 = utils.norm2( ei(1:3) );
    ei_ang_norm2 = utils.norm2( ei(4:6) );
    
    % If either limit is greater than the square of the threshold, then
    % rescale that portion of the error.
    if ei_lin_norm2 > opt.maxLinearErrorStep^2
        ei(1:3) = opt.maxLinearErrorStep * ei(1:3) / sqrt(ei_lin_norm2);
    end
    if ei_ang_norm2 > opt.maxAngularErrorStep^2
        ei(4:6) = opt.maxAngularErrorStep * ei(4:6) / sqrt(ei_ang_norm2);
    end
    
end