function bvp_parallel = get_BVP_parallel(bvp_parallel, y_vec, heading_vec)
%GET_BVP_PARALLEL Summary of this function goes here
%   Detailed explanation goes here
n_headings = find(heading_vec == pi/4);
init_states = [];
final_states = [];
final_params = [];
direction = [];
dirVec = [1, -1];
for l = 1:length(dirVec)
    dir = dirVec(l);
    for i = 1:n_headings
        th_i = heading_vec(i);
        init_state = [0,0,i,0,0];
        
        for j = 1:length(y_vec)
            
            final_state = [0,0, i,0,0];
            final_param = [-sin(th_i),cos(th_i), y_vec(j)];
            init_states = [init_states; init_state];
            final_states = [final_states; final_state];
            final_params = [final_params; final_param];
            direction = [direction; dir];
        end
        
        if(th_i > 0 && th_i < pi/4)
            for j = 1:length(y_vec)
                final_state = [0,0, i,0,0];
                final_param = [-sin(th_i),cos(th_i), -y_vec(j)];
                init_states = [init_states; init_state];
                final_states = [final_states; final_state];
                final_params = [final_params; final_param];
                direction = [direction; dir];
            end
        end
    end
end
bvp_parallel.initStates = init_states;
bvp_parallel.finalStates = final_states;
bvp_parallel.parallelParams = final_params;
bvp_parallel.direction = direction;
bvp_parallel.heading_vec = heading_vec;
