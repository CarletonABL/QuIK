function make(rid)
    % MAKE Compiles the matlab functions for codegen
    
    if nargin < 1
        rid = [];
    end
            
    IKdir = fileparts(mfilename('fullpath'));

    
    % Compiler options
    cfg = coder.config('mex');
    cfg.GenerateReport = true;
    cfg.ReportPotentialDifferences = false;
    cfg.EnableMexProfiling = false;
    cfg.ExtrinsicCalls = false;
    cfg.SaturateOnIntegerOverflow = false;
    cfg.IntegrityChecks = false;
    cfg.ResponsivenessChecks = false;
    cfg.EchoExpressions = false;
    cfg.MATLABSourceComments = true;
    
    % Robots to compile
    compile_robots = [robots.KUKA_KR6();
                      robots.KUKA_KR6_cal();
                      robots.jaco();
                      robots.KUKA_iiwa7();
                      robots.atlas()];

    for i = 1:numel(compile_robots)
        
        r = compile_robots(i);
        
        if ~isempty(rid) && ~strcmp(rid, r.id)
            continue;
        end
        
        fprintf("Compiling IK_matlab.IK_%s into mex...\n", r.id);

        % Function arguments
        ARGS = cell(3,1);
        ARGS{1} = coder.typeof(0, [4, 4, Inf], [0 0 1]); % Twt
        ARGS{2} = coder.typeof(0, [50, Inf], [1 1]); % Q0
        ARGS{3} = struct( ... % opt
                    'iterMax', coder.typeof(int32(0)), ... % iterMax
                    'algorithm', coder.typeof(int32(0)), ... % algorithm
                    'exitTol', coder.typeof(0), ... % exitTol
                    'minStepSize', coder.typeof(0), ... % gradExitTol
                    'relImprovementTol', coder.typeof(0), ... % relImprovementTol
                    'maxGradFails', coder.typeof(int32(0)), ... % maxGradFails
                    'maxGradFailsTotal', coder.typeof(int32(0)), ... % maxGradFails
                    'lambda2', coder.typeof(0), ... % Lambda parameter for damped newton
                    'maxLinearErrorStep', coder.typeof(0), ... % maxLinearErrorStep
                    'maxAngularErrorStep', coder.typeof(0), ... % maxAngularErrorStep
                    'armijoRuleSigma', coder.typeof(0), ... % sigma for line search
                    'armijoRuleBeta', coder.typeof(0) ... % beta for line search
                    ); 
    
        % Create up-to-date savefile of robot
        rb = IK_matlab.struct2rbtree(r);
    
        dir = pwd;
        cd( fullfile( IKdir, '..', '+robots', '+matlab') );
        rb.writeAsFunction( r.id );
        cd( dir );
        
        funcName = sprintf('IK_%s', r.id);
        mexName = [funcName, '_mex'];
    
        % Start codegen
        codegen(['+IK_matlab/', funcName], ...
            '-config', cfg, ...
            '-args', ARGS, ...
            '-o', fullfile(IKdir, mexName));
        
        fprintf("Done.\n");
        
    end

end