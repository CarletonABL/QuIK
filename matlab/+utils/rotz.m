function R = rotz(angle)
    % ROTZ Generates a rotation matrix along the z dimension.
    % Angle is in radians.
    %
    % R = utils.rotz(theta) returns the 3x3 rotation matrix for a
    % theta radian rotation about the z axis.
    %
    % See also utils.rotx, utils.roty, utils.rot
    
    R = utils.rot(angle, 3);
end