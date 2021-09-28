function A = structfield(S, field)
    % Extracts a field from a structure array
    
    A1 = cat(4, S.(field));
    szA = size( A1(:, :, :, 1) );
    szS = size( S );
    szA2 = [szA, szS];
    A2 = reshape(A1, szA2);
    A = squeeze(A2);
end