function [Q, Twt] = randomPose(r, N, opt)
    % Returns N random poses for a robot denoted by a structure r
    %
    % Q = randomPose(r, N)
    
    arguments
        r
        N
        opt.repeatable logical = true % Whether to reset the random seed before running
    end
    
    % Reset seed
    if opt.repeatable
        rng(1);
    end
    
    % Generate random poses
    Q = rand(r.DOF, N)*2*pi - pi;
    
    % Do forward kinematics on each sample, if requested
    if nargout >= 2
        Twt = zeros(4,4,N);
        for i = 1:N
            Twt_in = QuIK.FK(r, Q(:, i));
            Twt(:, :, i) = Twt_in(:, :, end);
        end
    end
    
end