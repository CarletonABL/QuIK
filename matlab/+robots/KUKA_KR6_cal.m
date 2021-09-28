function r = KUKA_KR6_cal()
    % KUKA_KR6 Returns a struct with DHx, Ttool and Tbase fields in it for
    % this robot.

    r = robots.KUKA_KR6;
    r.name = 'KUKA KR6 (Perturbed DH, 6-DOF)';
    r.id = 'KUKA_KR6_cal';
    
    % Perturb DH table
    perturbAmount = 0.01;
    rng(0); % Reset random seed so that this is repeatable.
    r.DHx(:, 1:3) = r.DHx(:, 1:3) + (rand(6,3) - .5)*2*perturbAmount;
    r.errorMask = int32(0:5);
end