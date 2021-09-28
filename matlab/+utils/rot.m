function R = rot(angle, axis)
    % ROT Generates a rotation matrix along a principle coordinate.
    % Angle is in radians. Axis is 1 for x, 2 for y, 3 for z.
    %
    % R = utils.rot(theta, axis) returns the 3x3 rotation matrix for a
    % theta radian rotation about axis.
    %
    % See also utils.rotx, utils.roty, utils.rotz
    
    c = permute(cos(angle(:)), [2 3 1]);
    s = permute(sin(angle(:)), [2 3 1]);
    N = numel(s);
    z = zeros(1,1,N);
    o = ones(1,1,N);

    switch axis
        case 1
            R = [o z z;
                 z c -s;
                 z s c];
        case 2
            R = [c z s;
                 z o z;
                 -s z c];
        case 3
            R = [c -s z;
                 s c z;
                 z z o];
        otherwise
            error("Axis must be 1-3 (for x, y and z)");
    end
    
end