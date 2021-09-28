function r = rbtree2struct( R, Ltool, Lstart )
    % Turns a matlab structure into a simple r structure including DH
    % table, etc
    %
    % r = rbtree2struct( R, Ltool, Lstart )
    %
    % Lstart is optional
    % If Lstart is given, then the chain MUST go through the "base link". 
    
    % Start, find the chain between the start and end points
    Bbase = R.Base;

    chain = searchLinks( Bbase, Ltool );
    chain = chain(2:end); % Ignore the base joint
    
    % Get the start joint, if applicable
    if nargin >= 3 && ~isempty(Lstart)
        chain2 = searchLinks( Bbase, Lstart );
        chain = [flipud(chain2(2:end)); chain];
    end
    
    % Remove fixed joints
    mask = arrayfun( @(b) ~strcmp(b.Joint.Type, 'fixed'), chain );
    chain = chain(mask);
    
    % Verify dataformat
    R.DataFormat = 'column';
    Q = R.homeConfiguration * 0;
    
    % Do forward kinematics
    jnts = vertcat(chain.Joint);
    nJnts = numel(jnts);
    T = zeros(4,4,nJnts);
    linkTypes = zeros(nJnts,1);
    for i = 1:nJnts
        T(:, :, i) = R.getTransform( Q, Bbase.Name, chain(i).Name );
        linkTypes(i) = ~strcmp(jnts(i).Type, 'revolute');
    end
        
    % Extract joint axis and origins
    z = squeeze(pagemtimes(T(1:3, 1:3, :), permute(cat(3, jnts.JointAxis), [2 1 3])));
    c = squeeze(T(1:3, 4, :));
    
    % Run autoDH algorithm
    DH = RU.autoDH(c, z);
    
    % Assemble results
    r.DHx = round([DH, linkTypes, ones(nJnts,1)], 10);
    r.Tbase = eye(4);
    r.Ttool = eye(4);
    r.DOF = nJnts;
end

function [chain, flag] = searchLinks( chain, name )
    % Searches for a link in a tree, recursively
    
    children = chain(end).Children;
    nChildren = numel(children);
    flag = false;
    for i = 1:nChildren
        % Build candidate chain
        chain_i = [chain; children{i}];
        
        if strcmp( children{i}.Name, name )
            flag = true;
            chain = chain_i;
            return;
        else
            % Search children's children
            [chain_i2, flag] = searchLinks( chain_i, name );
            if flag
                chain = chain_i2;
                return;
            end
        end
        
    end
    
end

function flag = hasParent( B )
    try
        a = B.Parent;
        flag = true;
    catch
        flag = false;
    end
end
function flag = hasChildren( B )
    try
        a = B.Children;
        flag = true;
    catch
        flag = false;
    end
end