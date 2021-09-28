function R = rotx(angle)
    % ROTX Generates a rotation matrix along the x dimension.
    % Angle is in radians.
    %
    % R = utils.rotx(theta) returns the 3x3 rotation matrix for a
    % theta radian rotation about the x axis.
    %
    % See also utils.rot, utils.roty, utils.rotz
    
    R = utils.rot(angle, 1);
end