function [Q, ec] = IK( r, Twt, Q0, opt )
    % IK A basic IK implementation of the QuIK, NR and BFGS algorithms.
    %
    % [Q, ec] = QuIK.IK( r, Twt, Q0, opt )
    %
    % Note! This algorithm runs slowly in native matlab code. It runs much
    % faster when it is auto-compiled into C-code using matlab codegen. To
    % do this, run "QuIK.make", then run "IK.IK_mex(_____)" rather than
    % "QuIK.IK(____)".
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

    %% Function setup
    DOF = size(r.DHx, 1);
    n = 6; % Number of constraints
    N = int32(size(Twt, 3));
    lt = logical( r.DHx(:, 5) );
    
    % Pre-init
    Q = zeros(DOF, N);
    ec = repmat(struct('e', zeros(6, 1), ...
                       'iter', int32(0), ...
                       'breakReason', int32(0)), ...
                [N, 1]);

    % Define some variables for codegen
    % These variables are redefined later, but the size must be determined
    % beforehand
    Ji = coder.nullcopy( zeros(DOF, n) );
    H = coder.nullcopy( zeros(DOF, n) );
    grad = coder.nullcopy( zeros(n, 1) );
    cost = 1e10;
            
    % Loop through each sample
    for k = int32(1):N
        %% Single Inverse Kinematics Solution
        
        % Setup sample
        Qi = Q0(:, k);
        ei = zeros(6,1);
        dQi = zeros(DOF, 1);
        iter_i = opt.iterMax; % Set to iterMax
        breakReason_i = int32(3); % Set to value that means max iterations reached.
        ei_prev_norm = 1e10;
        grad_fail_counter = int32(1);
                
        % Start iteration
        for i = int32(1):opt.iterMax
            
            if opt.algorithm ~= int32(3) || i == int32(1)
                % Get FK and error. For BFGS algorithm, this is already
                % calculated
                % Get FK
                Ti = QuIK.FK(r, Qi);

                % Get error
                ei = QuIK.hgtDiff( Ti(:, :, end), Twt(:, :, k) );
                
                % Get Jacobian
                Ji = QuIK.jacobian(r, Ti);
            end
            
            % Get norms
            ei_norm = sqrt(utils.norm2(ei));
            
            % Determine if breaking (error tolerance)
            if ei_norm <= opt.exitTol
                breakReason_i = int32(1); % Tolerance reached
                iter_i = i;
                break;
            end
            
            % Check relative improvement in error
            error_relImprovement = (ei_prev_norm - ei_norm) / ei_prev_norm;
            if error_relImprovement < opt.relImprovementTol
                grad_fail_counter = grad_fail_counter + int32(1);
                
                if grad_fail_counter > opt.maxGradFails
                    breakReason_i = int32(4); % Grad fails reached
                    iter_i = i;
                    break;
                end
            else
                grad_fail_counter = int32(1);
            end
                        
            % Store prev value
            ei_prev_norm = ei_norm;
            
            % Clamp error 
            ei = QuIK.clampMag(ei, opt);      
            
            % Get step
            switch opt.algorithm
                
                case int32(0)
                    %% Halley method (damped if lambda not 0)
                    
                    % First, get the newton step
                    dQi = - utils.lsolve( Ji, ei, opt);
                    
                    % A = 1/2 * H * dQ
                    Ai = QuIK.hessianProduct( lt, Ji, dQi/2 );

                    % Calculate the Halley correction
                    dQi = - utils.lsolve(Ji + Ai, ei, opt);
                    
                case int32(1)
                    %% Newton Raphson (damped if lambda not 0)
                    
                    % All algorithms require the newton step
                    dQi = - utils.lsolve( Ji, ei, opt);
                    
                case int32(2)
                    %% BFGS
                    
                    % Initialize H, cost and gradient
                    if i == 1
                        H = eye(n);
                        grad = Ji' * ei;
                        cost = 0.5*(ei'*ei);
                    end
                                        
                    % Line search
                    gamma = 1;
                    s0 = -H*grad;
                    
                    % Compute cost of step
                    [cost_next, ei_next, Ti_next] = BFGScost(Qi + gamma*s0, r, Twt(:, :, k));
                                        
                    % Line search loop
                    while cost - cost_next < -opt.armijoRuleSigma * grad'*(gamma*s0)
                        gamma = opt.armijoRuleBeta * gamma;

                        % Break if stepsize is small
                        if gamma < opt.minStepSize, break; end
                        
                        % Recalculate cost
                        [cost_next, ei_next, Ti_next] = BFGScost(Qi + gamma*s0, r, Twt(:, :, k));
                    end
                    
                    % Break out if step size is small
                    if gamma < opt.minStepSize
                        breakReason_i = int32(2);
                        iter_i = i;
                        break;
                    end
                    
                    % Once step size is decided, take that step size            
                    dQi = gamma*s0;
                    
                    % Update gradient
                    Ji_next = QuIK.jacobian(r, Ti_next);
                    grad_next = Ji_next' * ei_next;
                    
                    % Update search direction using damped BFGS
                    % y is the difference of gradients
                    y = grad_next - grad;
                    
                    % From http://ai.stanford.edu/~latombe/cs99k/2000/badler.pdf
                    % Zhao J, Badler NI. Inverse kinematics positioning
                    % using nonlinear programming for highly articulated
                    % figures. ACM Transactions on Graphics (TOG).
                    % 1994 Oct 1;13(4):313-36.
                    %
                    % If rho or delta are 0, then the update won't
                    % occur (better than introducing nan's into
                    % calculations)
                    sigma = gamma*s0;
                    rho = sigma'*y;
                    delta = y'*H*y;
                    if rho > delta && rho > eps
                        H = H + ((1 + delta/rho) * (sigma*sigma') - sigma*y'*H - H*y*sigma')/rho;
                    elseif delta > eps && rho > eps
                        H = H + (sigma*sigma')/rho - H*(y*y')*H/delta;
                    end

                    % Store gradient and cost for next time
                    grad = grad_next;
                    ei = ei_next;
                    Ti = Ti_next;
                    cost = cost_next;

                    
                                        
                otherwise
                    error("Invalid algorithm id");
                    
            end
            
            %% Apply step and check conditions
            
            % Apply change
            Qi = Qi + dQi;
            
            % Check minStepSize
            dQi_norm = utils.norm2(dQi);
            if dQi_norm < opt.minStepSize^2 && opt.algorithm ~= 4
                breakReason_i = int32(2); % grad tolerance reached
                iter_i = i;
                break;
            end

        end % End of inverse kinematics iterations
        
        % Store results
        Q(:, k) = Qi;
        ec(k).e = ei;
        ec(k).iter = iter_i;
        ec(k).breakReason = breakReason_i;
        
    end % End of sample iterations
    
end

%% Helper functions
function [cost, e, T] = BFGScost(Q, r, Twt)
    % Short code to return cost, e, and T for the BFGS algorithm
    T = QuIK.FK(r, Q);
    e = QuIK.hgtDiff( T(:, :, end), Twt );
    cost = 0.5*(e'*e);
end