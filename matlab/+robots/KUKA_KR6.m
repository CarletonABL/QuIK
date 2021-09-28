function r = KUKA_KR6()
    % KUKA_KR6 Returns a struct with DHx, Ttool and Tbase fields in it for
    % this robot.

    r.name = 'KUKA KR6 (6-DOF)';
    r.id = 'KUKA_KR6';
    r.DHx = [   0.025,	-pi/2,	0.183,	0,  0,  1;
                -0.315,	0,      0,      0,  0,  1;
                -0.035, pi/2,	0,      0,  0,  1;
                0,      -pi/2,	0.365,  0,  0,  1;
                0,      pi/2,	0,      0,  0,  1;
                0,      0,      0.08,	0,  0,  1];
            
    r.Ttool = [utils.roty(pi/2), [0.15 0.25 0.1]'; 0 0 0 1];
    r.Tbase = eye(4);
    r.DOF = size(r.DHx,1);
    r.nominalPose = [0;0;pi/2;0;pi/2;0];
    r.errorMask = int32(0:5);
end