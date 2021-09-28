function R = roty(angle)
    % ROTY Generates a rotation matrix along the y dimension.
    % Angle is in radians.
    %
    % R = utils.roty(theta) returns the 3x3 rotation matrix for a
    % theta radian rotation about the y axis.
    %
    % See also utils.rotx, utils.rot, utils.rotz
    
    R = utils.rot(angle, 2);
end