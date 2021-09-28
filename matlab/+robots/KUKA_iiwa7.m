function r = KUKA_iiwa7()
    % KUKA iiwa7 Returns a struct with DHx, Ttool and Tbase fields in it for
    % this robot.
    %
    % Data from https://link.springer.com/content/pdf/10.1007%2F978-981-15-3639-7.pdf
    % Dual Quaternion-Based Kinematic Modelling of Serial Manipulators
    % Dalvi et al, 2020

    r.name = 'KUKA iiwa7 R800 (7-DOF)';
    r.id = 'KUKA_iiwa7';
    r.DHx = [   0,  -pi/2,	0.34,	0,  0,  1;
                0,	pi/2,	0,      0,  0,  1;
                0,  -pi/2,	0.4,	0,  0,  1;
                0,	pi/2,	0,      0,  0,  1;
                0,	-pi/2,	0.4,	0,  0,  1;
                0,	pi/2,	0.4,	0,  0,  1;
                0,	0,      0,      0,  0,  1];
            
    r.Ttool = [utils.roty(pi/2), [0.15 0.25 0.1]'; 0 0 0 1];
    r.Tbase = eye(4);
    r.DOF = size(r.DHx,1);
    r.nominalPose = [0; pi/4; 0; pi/2; 0; pi/4; 0];
    r.errorMask = int32(0:5);
end