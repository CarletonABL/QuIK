function opt = config
    % CONFIG This file serves as a configuration for the libraries.
    %
    % You are required to define some options to be able to properly
    % compile the given code.
    
    % Eigen3Dir
    % To properly compile the code, an Eigen 3 installation is required.
    % This path will be given to the mex compiler with the -I flag.
    % https://eigen.tuxfamily.org/
    %
    % Example: opt.Eigen3Dir = '/path/to/eigen-3.4.0'
    % 
    % Path to the top level clone directory is sufficient - do not include
    % the final path to the internal header directory inside the clone.
    opt.Eigen3Dir = '';

end