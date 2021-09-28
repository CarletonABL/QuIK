function [Qstar, e, iter, breakReason] = IK_atlas( Twt, Q0, opt )
    % IK wrapper specifically for KR6 robot
    
    robot = robots.matlab.atlas;
    
    [Qstar, e, iter, breakReason] = IK_matlab.IK( robot, Twt, Q0, opt );
    
end