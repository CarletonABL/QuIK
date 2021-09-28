function [Q_star, e_star, iter, breakReason] = IK( r, Twt, Q0, opt )
    % IK Just a wrapper function that re-arranges the inputs and calls the
    % correct mex adapter function to the C++ library.
    %
    % Inputs:
    %   - r: A robot structure, with the following fields:
    %       * DHx: A DOFx6 array of the DH params in the following order: 
    %         [a_1  alpha_1    d_1   theta_1   lt_1    q_sign_1;
    %          :       :        :       :        :         :     
    %          an   alpha_n    d_n   theta_n   lt_n    q_sign_n];
    %  
    %           lt stands for "Link types", and should be true if joint is a
    %           prismatic joint, false otherwise. If column 6 is missing, will
    %           assume revolute (0).
    % 
    %           q_sign is a link direction number (-1 or 1). Allows you to
    %           change the sign of the joint variable. If column 7 is
    %           missing, will assume 1.
    %       * Tbase: [4x4] The transform between the 0th and world frame of the
    %           manipulator
    %       * Ttool: [4x4] The transform between the nth and tool frame of the
    %           manipulator
    %   - Twt: [4x4xN] A page matrix of desired poses of the robot.
    %   - Q0: [DOFxN] Initial guesses of the joint angles
    %   - opt: An options array for the inverse kinematics algorithm. This
    %       structure can be generated using the IKoptions field to fill in
    %       defaults.
    %       * iterMax [int32]: Maximum number of iterations of the
    %           algorithm. Default: int32(100)
    %       * algorithm [int32]: The algorithm to use
    %           0 - QuIK
    %           1 - Newton-Raphson or Levenberg-Marquardt
    %           2 - BFGS
    %           Default: int32(0).             
    %       * exitTol [double]: The exit tolerance on the norm of the
    %           error. Default: 1e-12.
    %       * minStepSize [double]: The minimum joint angle step size
    %           (normed) before the solver exits. Default: 1e-14.
    %       * relImprovementTol [double]: The minimum relative
    %           iteration-to-iteration improvement. If this threshold isn't
    %           met, a counter is incremented. If the threshold isn't met
    %           [maxGradFails] times in a row, then the algorithm exits.
    %           For example, 0.05 represents a minimum of 5% relative
    %           improvement. Default: 0.05.
    %       * maxGradFails [int32]: The maximum number of relative
    %           improvement fails before the algorithm exits. Default:
    %           int32(20).
    %       * lambda2 [double]: The square of the damping factor, lambda.
    %           Only applies to the NR and QuIK methods. If given, these
    %           methods become the DNR (also known as levenberg-marquardt)
    %           or the DQuIK algorithm. Ignored for BFGS algorithm. 
    %           Default: 0.
    %       * maxLinearErrorStep [double]: An upper limit of the error step
    %           in a single step. Ignored for BFGS algorithm. Default: 0.3.
    %       * maxAngularErrorStep [double]: An upper limit of the error step
    %           in a single step. Ignored for BFGS algorithm. Default: 1. 
    %       * armijoRuleSigma [double]: The sigma value used in armijo's
    %           rule, for line search in the BFGS method. Default: 1e-5
    %       * armijoRuleBeta [double]: The beta value used in armijo's
    %           rule, for line search in the BFGS method. Default: 0.5
    %       * rcondLimit [double]: Limit on the recipricol condition number
    %			of the Jacobian matrix. If the condition number dips below this
    %           the algorithm automatically exits. Default: 1e-8
    %
    % Outputs:
    %   - Q: [DOFxN] The solved joint angles
    %   - ec: (Nx1 struct array) The exit conditions of the solutions
    %       * e [6x1] The pose error at the final solution.
    %       * iter [int32] The number of iterations the algorithm took.
    %       * breakReason [int32] The reason the algorithm stopped.
    %               1 - Tolerance reached
    %               2 - Min step size reached
    %               3 - Max iterations reached
    %               4 - Grad fails reached
    %               5 - rcond limit reached, solution is likely singular
    
    % Reshape input for mex function
    Twt_reshape = utils.vertStack(Twt, 3);
    
    % Call mex-file
    [Q_star, e_star, iter, breakReason] = QuIK_cpp.IK_mex(r.DHx, r.Tbase, r.Ttool, Twt_reshape, Q0, opt);
    
    
end