function [bvp_obj, primVec] = generate_primitives(printFlag, bvp_obj, bvp_gen)
[nBVPs, ~] = size(bvp_obj.initStates);
primVec = [];
N = bvp_obj.N;
if printFlag
    figure;clf;hold on; grid on; axis equal
    
end
if(strcmp(bvp_obj.type, 'straight') )
    prev_heading = inf;
    prev_final_heading = inf;
    L = bvp_obj.opti.value(bvp_obj.modelP(1));
    primId = 0;
    head_tol = pi/4;
    for i = 1:nBVPs
        init_s = bvp_obj.initStates(i,:);
        final_s =bvp_obj.finalStates(i,:);
        init_heading_ID = init_s(3)-1;
        final_heading_ID = final_s(3)-1;
        init_s(3) = bvp_obj.heading_vec(init_s(3));
        final_heading = bvp_obj.heading_vec(final_s(3));
        if(final_heading > 3*pi/2)
            final_heading = final_heading - 2*pi;
        end
        final_s(3) = final_heading;
        dir = bvp_obj.direction(i);
        curr_heading = init_s(3);
        curr_final_heading = final_s(3);
        bvp_obj.opti.set_initial(bvp_obj.X(4,:),0)
        bvp_obj.opti.set_initial(bvp_obj.X(5,:),0)
        %Initialize
        if(dir == 1)
            bvp_obj.opti.set_initial(bvp_obj.X(1,:),linspace(init_s(1), final_s(1),N+1));
            bvp_obj.opti.set_initial(bvp_obj.X(2,:),linspace(init_s(2), final_s(2),N+1));
            bvp_obj.opti.set_initial(bvp_obj.X(3,:),linspace(curr_heading,curr_final_heading,N+1));
            % Final States
            bvp_obj.opti.set_value(bvp_obj.initStateP, init_s);
            bvp_obj.opti.set_value(bvp_obj.finalStateP, final_s);
            
        else
            bvp_obj.opti.set_initial(bvp_obj.X(1,:),linspace(final_s(1),init_s(1),N+1));
            bvp_obj.opti.set_initial(bvp_obj.X(2,:),linspace(final_s(2),init_s(2),N+1));
            bvp_obj.opti.set_initial(bvp_obj.X(3,:),linspace(final_s(3),init_s(3),N+1));
            % Final States
            bvp_obj.opti.set_value(bvp_obj.initStateP, final_s);
            bvp_obj.opti.set_value(bvp_obj.finalStateP, init_s);
        end
        try
            sol_i = bvp_obj.opti.solve();
            feasible = 1;
        catch
            warning("Infeasible problem, skipping this primitive")
            feasible = 0;
            pause;
        end
        if feasible
            if printFlag
                plot(sol_i.value(bvp_obj.X(1,:)), sol_i.value(bvp_obj.X(2,:)));
            end
            
            cost = bvp_obj.opti.value(bvp_obj.objective);
            if dir == 1
                prim_i = Primitive(primId, init_heading_ID, final_heading_ID, cost, sol_i.value(bvp_obj.X(1:4,:)), dir);
                
            else
                prim_i = Primitive(primId, final_heading_ID, init_heading_ID, cost, sol_i.value(bvp_obj.X(1:4,end:-1:1)), dir);
            end
            
            primVec = [primVec, prim_i];
            primId = primId + 2;
            prev_heading = curr_heading;
            prev_final_heading = curr_final_heading;
            bvp_obj.opti.set_initial(sol_i.value_variables);
        end
    end
else
    prev_heading = inf;
    prev_final_heading = inf;
    head_tol = atan(1);
    L = bvp_obj.opti_fwd.value(bvp_obj.modelP_fwd(1));
    isParallel = strcmp(bvp_obj.type, 'parallel');
    if isParallel
        primId = 1000;
    else
        primId = 5000;
    end
    for i = 1:nBVPs
        
        curr_dir = bvp_obj.direction(i);
        init_s = bvp_obj.initStates(i,:);
        final_s = bvp_obj.finalStates(i,:);
        init_heading_ID = init_s(3)-1;
        final_heading_ID = final_s(3)-1;
        init_s(3) = bvp_obj.heading_vec(init_s(3));
        final_heading = bvp_obj.heading_vec(final_s(3));
        if(final_heading > 3*pi/2)
            final_heading = final_heading - 2*pi;
        end
        final_s(3) = final_heading;
        
        if curr_dir == -1
            init_state = final_s;
            final_state = init_s;
            opti = bvp_obj.opti_rev;
            X = bvp_obj.X_rev;
            initStateP = bvp_obj.initStateP_rev;
            finalStateP = bvp_obj.finalStateP_rev;
            if isParallel
                parallelP = bvp_obj.parallelP_rev;
            end
            
        else
            init_state = init_s;
            final_state = final_s;
            opti = bvp_obj.opti_fwd;
            X = bvp_obj.X_fwd;
            initStateP = bvp_obj.initStateP_fwd;
            finalStateP = bvp_obj.finalStateP_fwd;
            if isParallel
                parallelP = bvp_obj.parallelP_fwd;
            end
        end
        curr_heading = init_state(3);
        curr_final_heading = final_state(3);
        
        
        % Re-Initialize states
        if curr_dir == 1
            opti.set_initial(X(1,:),linspace(0,cos(init_state(3))*L,N+1));
            opti.set_initial(X(2,:),linspace(0,sin(init_state(3))*L,N+1));
            opti.set_initial(X(3,:),linspace(init_state(3),final_state(3),N+1));
            opti.set_initial(X(4,:),0);
            opti.set_initial(X(5,:),0);
            
        else
            opti.set_initial(X(1,:),linspace(-cos(init_state(3))*L,0,N+1));
            opti.set_initial(X(2,:),linspace(-sin(init_state(3))*L,0,N+1));
            opti.set_initial(X(3,:),linspace(init_state(3),final_state(3),N+1));
            opti.set_initial(X(4,:),0);
            opti.set_initial(X(5,:),0)
            
        end
        %Set parameter values
        opti.set_value(initStateP(3), curr_heading);
        opti.set_value(finalStateP(3), curr_final_heading);
        if isParallel
            curr_params = bvp_obj.parallelParams(i,:);
            opti.set_value(parallelP, curr_params);
        end
        sol_i = opti.solve();
        
        if printFlag
            plot(sol_i.value(X(1,:)), sol_i.value(X(2,:)), '--');
        end
        %Choose one candidate
        bvp_gen.opti.set_value(bvp_gen.initStateP, init_state);
        bvp_gen.opti.set_value(bvp_gen.finalStateP, final_state);
        bvp_gen.opti.set_initial(sol_i.value_variables);
        if curr_dir == 1
            x_res = sol_i.value(X(1,end));
            y_res = sol_i.value(X(2,end));
        else
            x_res = sol_i.value(X(1,1));
            y_res = sol_i.value(X(2,1));
        end
        [x_cand, y_cand] = get_candidates(x_res,y_res, bvp_obj.disc, bvp_obj.type, final_state(3));
        bestCost = inf;
        n_cand = length(x_cand);
        opti.set_initial(sol_i.value_variables);
        if n_cand == 0
            X_sol = opti.value(X(1:4,:));
        else
            bvp_gen.opti.set_initial(sol_i.value_variables);
            cand_i = [];
            while isempty(cand_i)
                for j = 1:n_cand
                    if curr_dir == 1
                        bvp_gen.opti.set_value(bvp_gen.finalStateP(1), x_cand(j));
                        bvp_gen.opti.set_value(bvp_gen.finalStateP(2), y_cand(j));
                    else
                        bvp_gen.opti.set_value(bvp_gen.initStateP(1), x_cand(j));
                        bvp_gen.opti.set_value(bvp_gen.initStateP(2), y_cand(j));
                    end
                    try
                        sol_j = bvp_gen.opti.solve();
                        if(sol_j.value(bvp_gen.objective) < bestCost)
                            cand_i = sol_j;
                            bestCost = bvp_gen.opti.value(bvp_gen.objective);
                        end
                        bvp_gen.opti.set_initial(sol_j.value_variables);
                        
                    catch ME
                        warning("Infeasible candidate, proceed")
                        bvp_gen.opti.set_initial(sol_i.value_variables);
                        %pause
                    end
                end
                if ~isempty(cand_i)
                    X_sol = cand_i.value(bvp_gen.X(1:4,:));
                else
                    [x_cand, y_cand] = get_new_candidates(min(x_cand), max(x_cand), min(y_cand), max(y_cand), bvp_obj.disc);
                    n_cand = length(x_cand);
                end
            end
        end
        if curr_dir == 1
            prim_i = Primitive(primId, init_heading_ID, final_heading_ID, bestCost, X_sol, curr_dir);
        else
            prim_i = Primitive(primId, init_heading_ID, final_heading_ID, bestCost, X_sol(:,end:-1:1), curr_dir);
        end
        primVec = [primVec, prim_i];
        if printFlag
            plot(X_sol(1,:), X_sol(2,:));
        end
        prev_heading = curr_heading;
        prev_final_heading = curr_final_heading;
        primId = primId + 2;
    end
end

