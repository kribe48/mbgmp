%%

update_param = 0;

if ~update_param
    clear all; clc; close all;
    update_param = 0;
    import casadi.*
end
curr_cfg = 'cfg1';
tic
if ~update_param
    % Create BVP-objs
    bvp_head = struct;
    bvp_head.type = 'heading';
    bvp_par = struct;
    bvp_par.type = 'parallel';
    bvp_str = struct;
    bvp_str.type = 'straight';
    bvp_gen = struct;
    bvp_gen.type = 'generation';
    
    % Define parameters
    N = 50; % Number of control intervals
    % model parameters
    a_max = 45*(pi/180);
    L = 3;
    w_max = 1.5;
    u_max = 40;
    modelParams = [L, a_max, w_max, u_max];
    r = 1;
    % Define parameters and boundary value problems to be solved
    heading_vec_16 = [0, atan(1/2), atan(1), atan(2), pi/2, pi/2+atan(1/2),3*pi/4, pi/2+atan(2), pi, ...
        pi+atan(1/2), 5*pi/4, pi+atan(2), 3*pi/2, 3*pi/2+atan(1/2), 7*pi/4, 3*pi/2+atan(2)];
    heading_vec_8 = 0:pi/4:7*pi/4;
    switch curr_cfg
        case 'cfg1'
            heading_vec = heading_vec_8;
            delta_theta_max = 2;
            c_par = 1:3;
        case 'cfg2'
            heading_vec = heading_vec_16;
            delta_theta_max = 4;
            c_par = 1:5;
            
        case 'cfg3'
            heading_vec = heading_vec_16;
            delta_theta_max = 4;
            c_par = 1:8;
            
    end
    bvp_head = get_BVP_headings(bvp_head, heading_vec, delta_theta_max);
    bvp_str = get_BVP_straight(bvp_str, heading_vec, r);
    bvp_par = get_BVP_parallel(bvp_par, c_par, heading_vec);
    
    % Create opti objects
    bvp_head = setup_problem_car(bvp_head,modelParams, r, N);
    bvp_gen = setup_problem_car(bvp_gen,modelParams, r, N);
    bvp_par = setup_problem_car(bvp_par, modelParams, r, N);
    bvp_str = setup_problem_car(bvp_str, modelParams, r, N);
    % Set objectives
    T_weight = 1;
    smooth_weight = 0.5;
    objective_head_fwd = T_weight*bvp_head.T_fwd + smooth_weight*(sumsqr(bvp_head.U_fwd) + sumsqr(bvp_head.X_fwd(4,:)) + sumsqr(bvp_head.X_fwd(5,:)) );
    objective_head_rev = T_weight*bvp_head.T_rev + smooth_weight*(sumsqr(bvp_head.U_rev) + sumsqr(bvp_head.X_rev(4,:)) + sumsqr(bvp_head.X_rev(5,:)) );
    objective_par_fwd = T_weight*bvp_par.T_fwd + smooth_weight*(sumsqr(bvp_par.U_fwd) + sumsqr(bvp_par.X_fwd(4,:)) + sumsqr(bvp_par.X_fwd(5,:)) );
    objective_par_rev = T_weight*bvp_par.T_rev + smooth_weight*(sumsqr(bvp_par.U_rev) + sumsqr(bvp_par.X_rev(4,:)) + sumsqr(bvp_par.X_rev(5,:)) );
    objective_gen = T_weight*bvp_gen.T + smooth_weight*(sumsqr(bvp_gen.U) + sumsqr(bvp_gen.X(4,:)) + sumsqr(bvp_gen.X(5,:)) );
    objective_str = T_weight*bvp_str.T + smooth_weight*(sumsqr(bvp_str.U) + sumsqr(bvp_str.X(4,:)) + sumsqr(bvp_str.X(5,:)) );
    bvp_head.opti_fwd.minimize(objective_head_fwd);
    bvp_head.opti_rev.minimize(objective_head_rev);
    bvp_head.objective_fwd = objective_head_fwd;
    bvp_head.objective_rev = objective_head_rev;
    bvp_par.opti_fwd.minimize(objective_par_fwd);
    bvp_par.opti_rev.minimize(objective_par_rev);
    bvp_par.objective_rev = objective_par_fwd;
    bvp_par.objective_rev = objective_par_rev;
    bvp_gen.opti.minimize(objective_gen);
    bvp_gen.objective = objective_gen;
    bvp_str.opti.minimize(objective_str);
    bvp_str.objective = objective_str;
    
    % initialize states
    bvp_head.opti_fwd.set_initial(bvp_head.T_fwd,7);
    bvp_par.opti_fwd.set_initial(bvp_par.T_fwd,7);
    bvp_head.opti_rev.set_initial(bvp_head.T_rev,7);
    bvp_par.opti_rev.set_initial(bvp_par.T_rev,7);
    bvp_str.opti.set_initial(bvp_str.T,3);
end

% Update model Parameters
if(update_param)
    L_new = 4;
    bvp_head.opti_fwd.set_value(bvp_head.modelP_fwd(1), L_new);
    bvp_head.opti_rev.set_value(bvp_head.modelP_rev(1), L_new);
    bvp_par.opti_fwd.set_value(bvp_par.modelP_fwd(1), L_new);
    bvp_par.opti_rev.set_value(bvp_par.modelP_rev(1), L_new);
    bvp_str.opti.set_value(bvp_str.modelP(1), L_new);
    bvp_gen.opti.set_value(bvp_gen.modelP(1), L_new);
end
% Generate primitives!
printFlag = 0;
[bvp_head, primVecHead] = generate_primitives(printFlag, bvp_head, bvp_gen);
[bvp_par, primVecPar] = generate_primitives(printFlag, bvp_par, bvp_gen);

[bvp_str, primVecStr] = generate_primitives(printFlag, bvp_str);

primVecTot = [primVecHead, primVecPar, primVecStr];
% Mirror and Store Primitives
itStart = 0;
filePath = 'primitives/junk/';
primVec_mir = mirror_primitives(primVecTot);
store_primitives(primVec_mir,filePath,itStart);
toc
% Plot primitives!

figure; clf; hold on; grid on; axis equal;
p_vec = primVec_mir;
nPrim = length(p_vec);
lineW = 1;
for i = 1:nPrim
    if 1 || p_vec(i).fromHeadingId == 0
        p_vec(i).plot_primitive(1,lineW,'b');
        %pause
    end
end
