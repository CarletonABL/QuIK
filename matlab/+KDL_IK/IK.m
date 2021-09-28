% KDL_IK.IK Compute inverse kinematics using the KDL library.
% 
% Note! This is not a function file, it is only provided as a help file. To
% properly run this code, you must compile a mex-file using KDL_IK.make.
%
% 
%  Inputs:
%  		DHx: 6 x 6 (a alpha d theta linktype Qsign),
% 		Tbase: 4 x 4 (base transform)
%  		Ttool: 4 x 4 (tool transform)
%  		Twt: 4N x 4 (vertically stacked samples)
% 		Q0: 6 x n, Initial guesses
%  		opt: struct with fields
%  			- iterMax
%  			- algorithm
%  			- exitTol
%  			- gradExitTol
%  			- relImprovement
%  			- maxGradFails
%  			- lambda2
%  			- linAlg
%  			- maxLinearErrorStep
%  			- maxAngularErrorStep
%  Outputs:
%  		Q_star: 6 x N, solved angles
% 		e_star: double, N, error at solved angles
% 		breakReason: int, N, reason for breaking.
%