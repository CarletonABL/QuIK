function [Qstar, e, iter, breakReason] = IK_KUKA_iiwa7( Twt, Q0, opt )
    % IK wrapper specifically for KR6 robot
    
    robot = robots.matlab.KUKA_iiwa7;
    
    [Qstar, e, iter, breakReason] = IK_matlab.IK( robot, Twt, Q0, opt );
    
end