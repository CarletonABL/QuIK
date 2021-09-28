function r = jaco()
    % jaco Returns a struct with DHx, Ttool and Tbase fields in it for
    % this robot.

    r.name = 'Kinova Jaco (Nonspherical Wrist, 6-DOF)';
    r.id = 'jaco';
    D1 = 0.2755;
    D2 = 0.41;
    D3 = 0.2073;
    D4 = 0.0741;
    D5 = 0.0741;
    D6 = 0.16;
    e2 = 0.0098;
    aa = deg2rad(60);
    sa = sin(aa);
    s2a = sin(2*aa);
    d4b = D3 + (sa/s2a)*D4;
    d5b = (sa/s2a)*D4 + (sa/s2a)*D5;
    d6b = (sa/s2a)*D5 + D6;
    
    r.DHx = [   0,  pi/2,   D1,       0,  0,  1;
                D2,	pi,     0,        0,  0,  1;
                0,  pi/2,	-e2,      0,  0,  1;
                0,  2*aa,	-d4b,     0,  0,  1;
                0,  2*aa,	-d5b,     0,  0,  1;
                0,  pi,     -d6b,     0,  0,  1];
            
    r.Ttool = [utils.roty(pi/2), [0.15 0.25 0.1]'; 0 0 0 1];
    r.Tbase = eye(4);
    r.DOF = size(r.DHx,1);
    r.nominalPose = [0;pi/4;-1*pi/4;pi/2;2*pi/3;pi];
    r.errorMask = int32(0:5);
end