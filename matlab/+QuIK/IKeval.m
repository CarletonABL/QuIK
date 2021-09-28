function [e, c] = IKeval(r, Q, Q_star )
    % EVAL Returns an error vector of two joint values.
    % Used to evaluate IK performance
    %
    % [e, c] = IKeval(r, Q1, Q2 )
    
    N = size(Q, 2);
    
    % Calculate error
    e = zeros(6, N);
    c = zeros(N, 1);
    if coder.target('matlab')
        assert( all(size(Q) == size(Q_star)), "Input joints must be same size!");
        for i = 1:N
            Twt1i = QuIK.FK( r, Q(:, i) );
            Twt2i = QuIK.FK( r, Q_star(:, i) );
            Ji = QuIK.jacobian( r, Twt1i );
            c(i) = cond(Ji);
            e(:, i) = QuIK.hgtDiff( Twt1i(:, :, end), Twt2i(:, :, end) );
        end
    else
        parfor i = 1:N
            Twt1i = QuIK.FK( r, Q(:, i) );
            Twt2i = QuIK.FK( r, Q_star(:, i) );
            Ji = QuIK.jacobian( r, Twt1i );
            c(i) = cond(Ji);
            e(:, i) = QuIK.hgtDiff( Twt1i(:, :, end), Twt2i(:, :, end) );
        end
    end
    
end