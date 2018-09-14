function bvp_obj = setup_problem_car(bvp_obj,modelParameters, disc, N)
%INITIALIZE_OPT_CAR Summary of this function goes here
%   Detailed explanation goes here
if(strcmp(bvp_obj.type, 'heading') || strcmp(bvp_obj.type, 'parallel') )
    [opti_fwd, X_fwd, U_fwd, T_fwd, modelParams_fwd, initStateParams_fwd, finalStateParams_fwd] = setup_opti_car(modelParameters, N);
    [opti_rev, X_rev, U_rev, T_rev, modelParams_rev, initStateParams_rev, finalStateParams_rev] = setup_opti_car(modelParameters, N);
    
    % ---- initial conditions --------
    opti_fwd.subject_to(X_fwd(:,1) == initStateParams_fwd);
    opti_rev.subject_to(X_rev(:,N+1) == finalStateParams_rev);
    % ---- Final conditions ------------
    useFinal = [0,0,1,1,1];
    parallelParams_fwd = [];
    parallelParams_rev = [];
    if(strcmp(bvp_obj.type, 'parallel') )
        % Restrict the final (x,y) to a line, k1*x + k2*y = m
        parallelParams_fwd = opti_fwd.parameter(3);
        parallelParams_rev = opti_rev.parameter(3);
        opti_fwd.subject_to(parallelParams_fwd(1)*X_fwd(1,N+1) + parallelParams_fwd(2)*X_fwd(2,N+1) == parallelParams_fwd(3));
        opti_rev.subject_to(parallelParams_rev(1)*X_rev(1,1) + parallelParams_rev(2)*X_rev(2,1) == parallelParams_rev(3));
    end
    for i=1:5
        if(useFinal(i) == 1)
            opti_fwd.subject_to(X_fwd(i,N+1) == finalStateParams_fwd(i));
            opti_rev.subject_to(X_rev(i,1) == initStateParams_rev(i));
        end
    end
    
    
    
    bvp_obj.opti_fwd = opti_fwd;
    bvp_obj.opti_rev = opti_rev;
    bvp_obj.X_fwd = X_fwd;
    bvp_obj.X_rev = X_rev;
    bvp_obj.U_fwd = U_fwd;
    bvp_obj.U_rev = U_rev;
    bvp_obj.T_fwd = T_fwd;
    bvp_obj.T_rev = T_rev;
    bvp_obj.modelP_fwd = modelParams_fwd;
    bvp_obj.modelP_rev = modelParams_rev;
    bvp_obj.initStateP_fwd = initStateParams_fwd;
    bvp_obj.initStateP_rev = initStateParams_rev;
    bvp_obj.finalStateP_fwd = finalStateParams_fwd;
    bvp_obj.finalStateP_rev = finalStateParams_rev;
    bvp_obj.parallelP_fwd = parallelParams_fwd;
    bvp_obj.parallelP_rev = parallelParams_rev;
    bvp_obj.N = N;
    bvp_obj.disc = disc;
else
    [opti, X, U, T, modelParams, initStateParams, finalStateParams] = setup_opti_car(modelParameters, N);
    % ---- initial conditions --------
    opti.subject_to(X(:,1) == initStateParams);
    % ---- Final conditions
    opti.subject_to(X(:,N+1) == finalStateParams);
    
    bvp_obj.opti = opti;
    bvp_obj.X = X;
    bvp_obj.U = U;
    bvp_obj.T = T;
    bvp_obj.modelP = modelParams;
    bvp_obj.initStateP = initStateParams;
    bvp_obj.finalStateP = finalStateParams;
    bvp_obj.N = N;
    bvp_obj.disc = disc;
end

