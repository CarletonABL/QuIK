function e = hgtDiff( T1, T2 )
    % HGTDIFF Calculates the error vector between two homogenious
    % transforms T1 and T2.
    %
    % e = hgtDiff(T1, T2);
    %
    % Algorithm used is as described in
    %       [1] T. Sugihara, “Solvability-Unconcerned Inverse Kinematics
    %       by the Levenberg–Marquardt Method,” IEEE Trans. Robot.,
    %       vol. 27, no. 5, pp. 984–991, Oct. 2011.
    
     % Extract rotation matrices
    R1 = T1(1:3, 1:3); % n
    R2 = T2(1:3, 1:3); % d
    R = R1*R2';
    
    % Get cartesian position vectors
    d1 = T1(1:3, 4);
    d2 = T2(1:3, 4);
    
    % Implementation by matlab
    if isdiag(R)
        % Diagonal elements are either [1 1 1], [-1 1 -1], [1 -1 -1] or [-1
        % -1 1], since R must be orthogonal and det(R) = 1.
        % If the diagonal is [1 1 1], the error is zero. Otherwise, its 
        % pi/2 * (diag(R)+1)
        if all( diag(R) > 0 )
            w = zeros(3,1);
        else
            w = pi/2 * (diag(R)+1);
        end
    else
        l = [R(3,2) - R(2,3);
             R(1,3) - R(3,1);
             R(2,1) - R(1,2)];
        l_norm = norm(l);
        if l_norm < 1e-12
            w = 1/(trace(R) - 1) * l;
        else
            w = atan2( l_norm, trace(R) - 1 ) / l_norm * l;
        end
    end

    % Concatenate
    e = vertcat( d1 - d2 , w );
    
end