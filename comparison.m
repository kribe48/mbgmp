%% For comparison using generation in Pivtoriaiko
tic
bvp_comp = struct;
bvp_comp.type = 'comp';
bvp_comp.initStates = bvp_head.initStates;
bvp_comp.finalStates = bvp_head.finalStates;
bvp_comp.direction = bvp_head.direction;
bvp_comp.heading_vec = heading_vec;

bvp_comp = setup_problem_car(bvp_comp, modelParams, disc, N);

objective_comp = T_weight*bvp_comp.T + smooth_weight*(sumsqr(bvp_comp.U) + sumsqr(bvp_comp.X(4,:)) + sumsqr(bvp_comp.X(5,:)) );
bvp_comp.opti.minimize(objective_comp);
bvp_comp.objective = objective_comp;


bvp_comp.opti.set_initial(bvp_comp.T,3);

% Solve problems
printFlag = 0;
[bvp_comp, primVecComp] = generate_primitives_comparison(printFlag, bvp_comp);
toc

