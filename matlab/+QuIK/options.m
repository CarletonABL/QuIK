function opt = options( opt )
    % IK.OPTIONS Builds a structure that can be used as an input to the
    % inverse kinematics solver.
    %
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
    
    arguments
        opt.iterMax (1,1) int32 {mustBeReal,mustBeNonNan,mustBePositive} = 200
        opt.algorithm (1,1) int32 {mustBeMember(opt.algorithm,[0, 1, 2, 3])} = int32(0)
        opt.exitTol (1,1) double {mustBeReal,mustBeNonnegative,mustBeNonNan} = 1e-12
        opt.minStepSize (1,1) double {mustBeReal,mustBeNonnegative,mustBeNonNan} = 1e-14
        opt.relImprovementTol (1,1) double {mustBeReal, mustBePositive,mustBeNonNan,mustBeInRange(opt.relImprovementTol,0,1,'inclusive')} = 0.05
        opt.maxGradFails (1,1) int32 {mustBeReal,mustBePositive,mustBeNonNan} = int32(10)
        opt.maxGradFailsTotal (1,1) int32 {mustBeReal,mustBePositive,mustBeNonNan} = int32(80)
        opt.lambda2 (1,1) double {mustBeReal,mustBeNonnegative,mustBeNonNan,mustBeFinite} = 0
        opt.maxLinearErrorStep (1,1) double {mustBeReal,mustBePositive,mustBeNonNan} = 0.34
        opt.maxAngularErrorStep (1,1) double {mustBeReal,mustBePositive,mustBeNonNan} = 1
        opt.armijoRuleSigma (1,1) double {mustBeReal,mustBeInRange(opt.armijoRuleSigma,0,1,'exclusive'),mustBeNonNan} = 1e-5
        opt.armijoRuleBeta (1,1) double {mustBeReal,mustBeInRange(opt.armijoRuleBeta,0,1,'exclusive'),mustBeNonNan} = 0.5
    end
    
end
