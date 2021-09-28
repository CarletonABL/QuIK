function make()
    % QuIK_cpp.MAKE Makes a mex-file of the CPP implementation of the QuIK c++
    % codebase.
    %
    % To run this function, you must have matlab coder installed. You must
    % have also configured the /matlab/config.m file to point to a valid
    % installation of Eigen 3.
    
    IKdir = utils.QuIK_root;
    opt = config;
    
    assert(~isempty( opt.Eigen3Dir ), 'You must specify a path to an Eigen 3 directory!');
    
    disp("Compiling QuIK c++ library into a mex-file...");
    
    % Define library folders and includes
    srcFolder = fullfile(IKdir, 'C++', 'QuIK');
    adapterFile = fullfile(srcFolder, 'IK_mexAdapter.cpp');
    
    % Compile
    mexFile = fullfile(IKdir, 'matlab', '+QuIK_cpp', 'IK_mex');
    mex(adapterFile, ...
        '-R2018a', ...
        ['-I', opt.Eigen3Dir], ...
        ['-I', srcFolder], ...
        ['-I', fullfile(srcFolder, 'Robot')], ...
        ['-I', fullfile(srcFolder, 'IK')], ...
        '-output', mexFile);
    
end