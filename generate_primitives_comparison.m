function [bvp_obj, primVec] = generate_primitives_comparison(printFlag, bvp_obj)
[nBVPs, ~] = size(bvp_obj.initStates);
primVec = [];
N = bvp_obj.N;
if printFlag
    figure;clf;hold on; grid on; axis equal
    
end
primId = 5000;
for i = 1:nBVPs
    feasible = 0;
    init_s = bvp_obj.initStates(i,:);
    final_s =bvp_obj.finalStates(i,:);
    init_heading_ID = init_s(3)-1;
    final_heading_ID = final_s(3)-1;
    init_s(3) = bvp_obj.heading_vec(init_s(3));
    final_heading = bvp_obj.heading_vec(final_s(3));
    dir = bvp_obj.direction(i);
    heading = init_s(3);
    %Make sure correct diff
    if(abs(heading-final_heading) > pi )
        final_heading = final_heading - 2*pi;
    end
    final_s(3) = final_heading;
    for r=1:50
        if feasible
            break;
        end
        [x_cand, y_cand] = get_L_inf_candidates(r, dir);
        n_cand = length(x_cand);
        for j=1:n_cand
            x_f = x_cand(j);
            y_f = y_cand(j);
            final_s(1) = x_f;
            final_s(2) = y_f;
            %Initialize
            bvp_obj.opti.set_initial(bvp_obj.X(4,:),0)
            bvp_obj.opti.set_initial(bvp_obj.X(5,:),0)
            if(dir == 1)
                bvp_obj.opti.set_initial(bvp_obj.X(1,:),linspace(0,x_f,N+1));
                bvp_obj.opti.set_initial(bvp_obj.X(2,:),linspace(0,y_f,N+1));
                bvp_obj.opti.set_initial(bvp_obj.X(3,:),linspace(init_s(3),final_s(3),N+1));
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
                warning("Infeasible problem, search for next")
                feasible = 0;
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
                break;
            end
        end
    end
end

end


