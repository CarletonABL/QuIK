function n2 = norm2(v, dim)
    % NORM2 returns the squared norm of a vector v. This is faster than
    % just calculating the norm since the square root does not need to be
    % performed.
    
    if nargin < 2 || isempty(dim)
        dim = 1;
    end
    
    coder.inline('always');
    
    n2 = sum(v.^2, dim);
    
end