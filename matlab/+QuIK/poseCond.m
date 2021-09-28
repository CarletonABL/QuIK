function c = poseCond( r, q )

    % Helper function to return the condition number of a given pose
    T = QuIK.FK( r, q );
    J = QuIK.jacobian(r, T);
    c = cond( J );
    
end