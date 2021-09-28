function T = FK(r, Q)
    % FK Calculates the forward kinematics of the robot from the DH
    % parameters.
    %
    % T = RU.FK(r, Q)
    %
    % INPUTS:
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
    %   - Q: [DOFxN] matrix of generalized joint coordinates to
    %           evaluate the forward kinematics at.
    %
    % OUTPUTS:
    %   -T: [4x4xN] A page matrix of each frame of the robot.
    
    %% PARSE INPUTS
    
    DOF = size(r.DHx, 1);
    
    assert(size(Q, 1) == DOF && size(Q, 3) == 1, "Q must be a DOF x N matrix of joint angles");
    
    %% START FUNCTION
        
    DH_j = r.DHx(:, 1:4);
    DH_j(:, 3) = DH_j(:, 3) + Q.*( r.DHx(:, 5) ) .* r.DHx(:, 6);
    DH_j(:, 4) = DH_j(:, 4) + Q.*( 1 - r.DHx(:, 5) ) .* r.DHx(:, 6);
    
    T = zeros(4, 4, DOF+1);

    % Loop through frames
    for k = 1:DOF
        
        DH_jk = DH_j(k, :);

        sti = sin(DH_jk(4)); cti = cos(DH_jk(4));
        sai = sin(DH_jk(2)); cai = cos(DH_jk(2));
        ai = DH_jk(1);
        di = DH_jk(3);
                
        Ak =   [cti,	-sti*cai,	sti*sai,	ai*cti;
                sti,    cti*cai,    -cti*sai,   ai*sti;
                0,      sai,        cai,        di;
                0,      0,          0,          1];
        
        % Reduce
        if k > 1
            T(:, :, k) = T(:, :, k-1) * Ak;
        else
            T(:, :, k) = r.Tbase * Ak;
        end

    end
    
    % Assign tool frame
    T(:, :, DOF+1) = T(:, :, DOF) * r.Ttool;
        
end

