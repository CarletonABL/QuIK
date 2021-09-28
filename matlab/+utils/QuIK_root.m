function path = QuIK_root()
    % QUIK_ROOT Returns the path to the root of the QuIK package

    cdir = fileparts(mfilename('fullpath'));
    path = fullfile(cdir, '..', '..');

end