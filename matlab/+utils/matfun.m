function B = matfun(func, A, dim, doSqueeze)
    % MATFUN Identical functionality to arrayfun, but allows you to specify
    % which dimension the iteration should occur along. Code will be
    % parallized, if possible (done if a parallel pool is running, or if in
    % a matlab coder environment).
    %
    % B = matfun(func, A, dim)
    % B = matfun(func, A, dim, true) will squeeze the answer (remove any
    % singleton dimensions)
    
    dimList = 1:4;
    
    % Permute A so that the scan dimension is in dimension 4.
    reorderDims = [dimList( dimList~=dim ), dim];
    Aperm = permute(A, reorderDims);
    
    % Now loop along this dimension, with preallocation
    N = size(Aperm, 4);

    B1 = func( Aperm(:, :, :, 1) );
    sz = [size(B1,1), size(B1,2), size(B1,3)];
    
    Bperm = zeros([sz, N]);
    Bperm(:, :, :, 1) = B1;
    
    % Loop
    if coder.target('matlab') || utils.autoPar
        for i = 2:N
            Bperm(:, :, :, i) = func(Aperm(:, :, :, i));
        end
    else
        parfor i = 2:N
            Bperm(:, :, :, i) = func(Aperm(:, :, :, i));
        end
    end
    
    % Permute back & squeeze if necessary    
    if nargin >= 4 && doSqueeze
        B = squeeze(ipermute(Bperm, reorderDims));
    else
        B = ipermute(Bperm, reorderDims);
    end

    
end