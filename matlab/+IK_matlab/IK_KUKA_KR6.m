function [Qstar, e, iter, breakReason] = IK_KR6( Twt, Q0, opt )
    % IK wrapper specifically for KR6 robot
    
    robot = robots.matlab.KUKA_KR6;
    
    [Qstar, e, iter, breakReason] = IK_matlab.IK( robot, Twt, Q0, opt );
    
end