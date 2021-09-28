function Q0 = perturbPose( Q, eSize, opt )
    % Perturbs a pose by a given amount
    %
    % Q0 = perturbPose( Q, eSize )
    
    arguments
        Q
        eSize % Sigma parameter, e.g. the perturbation amount
        opt.repeatable logical = true % Whether to reset the random seed before running
    end
    
    % Reset seed
    if opt.repeatable
        rng(1);
    end
    
    Qpert = rand(size(Q)) - 0.5;
    Qpert_n = Qpert ./ mean( abs(Qpert), 1 );
    Q0 = Q + Qpert_n .* eSize;
    
end