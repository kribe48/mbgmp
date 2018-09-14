function bvp_straight = get_BVP_straight(bvp_straight, heading_vec, disc)
%GET_BVP_STRAIGHT Summary of this function goes here
%   Detailed explanation goes here
n_heading = find(heading_vec == pi/4);
init_states = []; final_states = []; direction = [];
dirVec = [1, -1];
for j = 1:2
    dir = dirVec(j);
    for i = 1:n_heading
        init_state = [0,0,i,0,0];
        final_xy = get_xy(heading_vec(i),disc, dir);
        final_state = [final_xy,i,0,0];
        
        init_states = [init_states; init_state];
        final_states = [final_states; final_state];
        direction = [direction; dir];
    end
end

bvp_straight.initStates = init_states;
bvp_straight.finalStates = final_states;
bvp_straight.direction = direction;
bvp_straight.heading_vec = heading_vec;
end

