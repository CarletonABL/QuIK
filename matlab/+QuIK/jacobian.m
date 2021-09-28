function J = jacobian(r, T)
    % JACOBIAN Calculates the geometric jacobian of the robot from the DH
    % parameters.
    %
    % J = jacobian(r, T)
    %
    % INPUTS:
    %   - r: A robot structure, which must minimally include the following
    %       fields:
    %       * DHx: A DOFx6 array of the DH params in the following order: 
    %             [a_1  alpha_1    d_1   theta_1   lt_1    q_sign_1;
    %              :       :        :       :        :         :     
    %             an   alpha_n    d_n   theta_n   lt_n    q_sign_n];
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
    %   -T: [4x4xN] A page matrix of each frame of the robot.
    %
    % OUTPUTS
    %   -J:     The jacobian matrix or matrices. Size will be 6xDOFxNxP,
    %           where N is the number of samples given, and P is DOF, if -1
    %           is request for frame (e.g. return all frames), or a
    %           singleton dimension if an actual frame is requested (e.g.
    %           frame = 6).
    
    %% PARSE INPUTS
    
    DOF = size(r.DHx, 1);
    
    %% RUN FUNCTION
        
    %  Extract the positions of frames and the unit z vectors
    z = [r.Tbase(1:3, 3), permute(T(1:3, 3, :), [1 3 2])];
    o = [r.Tbase(1:3, 4), permute(T(1:3, 4, :), [1 3 2])];
    
    % Cast 6th column of DH as link types
    lt = abs(r.DHx(:, 5)) > 0.5;
    
    % Pre-allocate space for results
    J = zeros(6, DOF);
                   
    % Trimmed linktypes is useful for masks
    prisMask = lt;
    revMask = ~prisMask;

    % Assign prismatic joints
    if any(prisMask)
        % Jvi = z_{i-1}
        J(1:3, prisMask, :) = z(:, prisMask);
    end

    % Assign revolute joints
    if any(revMask)
        % Select out z_{i-1} axes
        z_rev = z(:, revMask);

        % Jwi = z_{i-1}
        J(4:6, revMask, :) = z_rev;

        % Jvi = z_{i-1} x (o_n - o_{i-1}
        o_n = o(:, end);
        J(1:3, revMask, :) = cross( z_rev, repmat(o_n, [1 nnz(revMask)]) - o(:, revMask) );
    end
            
end