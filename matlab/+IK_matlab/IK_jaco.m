function [Qstar, e, iter, breakReason] = IK_jaco( Twt, Q0, opt )
    % IK wrapper specifically for KR6 robot
    
    robot = robots.matlab.jaco;
    
    [Qstar, e, iter, breakReason] = IK_matlab.IK( robot, Twt, Q0, opt );
    
end