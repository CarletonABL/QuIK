function [Qstar, e, iter, breakReason] = IK_KUKA_KR6_cal( Twt, Q0, opt )
    % IK wrapper specifically for KR6 robot
    
    robot = robots.matlab.KUKA_KR6_cal;
    
    [Qstar, e, iter, breakReason] = IK_matlab.IK( robot, Twt, Q0, opt );
    
end