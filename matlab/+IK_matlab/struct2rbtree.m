function rb = struct2rbtree(r)
    % STRUCT2RBTREE Converts a struct to a matlab rigidBodyTree
    
    rb = rigidBodyTree('dataformat', 'column', 'MaxNumBodies', 20);

    r.DOF = size(r.DHx, 1);
    
    % Base transform
    baseBody = rigidBody(sprintf('base2'));
    baseJnt = rigidBodyJoint('base2', 'fixed');
    baseJnt.setFixedTransform( r.Tbase );
    baseBody.Joint = baseJnt;
    rb.addBody(baseBody, 'base');
    
    % Add links
    body = cell(1, r.DOF);
    jnt = cell(1, r.DOF);
    for i = 1:r.DOF
        % Make body
        body{i} = rigidBody(sprintf('L%d', int32(i)));
        
        % Make joint
        if (r.DHx(i,5) > 0.5), jntType = 'prismatic'; else, jntType = 'revolute'; end
        jnt{i} = rigidBodyJoint(sprintf('J%d', int32(i)), jntType);
        jnt{i}.setFixedTransform(r.DHx(i, 1:4), 'dh');
        body{i}.Joint = jnt{i};
        
        % Add to rbtree
        if i > 1
            rb.addBody( body{i}, rb.Bodies{end}.Name );
        else
            rb.addBody( body{i}, 'base2' );
        end
    end
   
    % Tool transform
    toolBody = rigidBody(sprintf('tool'));
    toolJnt = rigidBodyJoint('tool', 'fixed');
    toolJnt.setFixedTransform( r.Ttool );
    toolBody.Joint = toolJnt;
    rb.addBody(toolBody, rb.Bodies{end}.Name);
    
end

