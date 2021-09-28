function make(functions)
    % Compiles a c-code based MEX file for the important functions of the
    % IK package
    %
    % IK.make()
    % IK.make(functionName)
    
    % Fill in default args
    if nargin < 1 || isempty(functions)
        functions = 'all';
    end
    
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
    
    IKdir = fileparts(mfilename('fullpath'));
    
    %% Build IK function
    if any( strcmp( functions, 'IK' ) ) || any( strcmp( functions, 'all' ) )
        
        disp("Compiling QuIK.IK into mex...");
        
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
        
        % Function arguments
        ARGS = cell(4,1);
        ARGS{1} = struct('DHx', coder.typeof(0, [20 6],[1 0]), ...
                         'Tbase', coder.typeof(0, [4 4],[0 0]), ...
                         'Ttool', coder.typeof(0, [4 4],[0 0]));
        ARGS{2} = coder.typeof(0, [4, 4, Inf], [0 0 1]); % Twt
        ARGS{3} = coder.typeof(0, [20, Inf], [1 1]); % Q0
        ARGS{4} = struct( ... % opt
                    'iterMax', coder.typeof(int32(0)), ... % iterMax
                    'algorithm', coder.typeof(int32(0)), ... % algorithm
                    'exitTol', coder.typeof(0), ... % exitTol
                    'minStepSize', coder.typeof(0), ... % gradExitTol
                    'relImprovementTol', coder.typeof(0), ... % relImprovementTol
                    'maxGradFails', coder.typeof(int32(0)), ... % maxGradFails
                    'lambda2', coder.typeof(0), ... % Lambda parameter for damped newton
                    'maxLinearErrorStep', coder.typeof(0), ... % maxLinearErrorStep
                    'maxAngularErrorStep', coder.typeof(0), ... % maxAngularErrorStep
                    'armijoRuleSigma', coder.typeof(0), ... % sigma for line search
                    'armijoRuleBeta', coder.typeof(0) ... % beta for line search
                    ); 
        
        % Codegen
        codegen('+QuIK/IK', '-config', cfg, '-args', ARGS, '-o', fullfile(IKdir, 'IK_mex'));        
                
        disp("Done.");

    end
    
    %% IKeval
    
    if any( strcmp( functions, 'IKeval' ) ) || any( strcmp( functions, 'all' ) )
        
        disp("Compiling QuIK.IKeval into mex...");
        
        % Compiler options
        cfg = coder.config('mex');
        cfg.GenerateReport = true;
        cfg.ReportPotentialDifferences = false;
        
        % Function arguments
        ARGS = cell(3,1);
        ARGS{1} = struct('DHx', coder.typeof(0, [20 6],[1 0]), ...
                         'Tbase', coder.typeof(0, [4 4],[0 0]), ...
                         'Ttool', coder.typeof(0, [4 4],[0 0]));
        ARGS{2} = coder.typeof(0, [50, Inf], [1 1]); % Q1
        ARGS{3} = coder.typeof(0, [50, Inf], [1 1]); % Q2
        
        % Codegen
        codegen('+QuIK/IKeval.m', '-config', cfg, '-args', ARGS, '-o', fullfile(IKdir, 'IKeval'));        
                
        disp("Done.");

    end
   
end

