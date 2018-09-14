function bvp_headings = get_BVP_headings(bvp_headings, headings,delta)

n_headings = find(headings == pi/4);
initStates = [];
finalStates = [];
direction = [];
dirVec = [1,-1];
for l = 1:length(dirVec)
    dir = dirVec(l);
    for i = 1:n_headings
        initState_i = [0,0,i,0,0];
        for j = 1:delta
            finalState_ij = [0,0,i+j,0,0];
            initStates = [initStates; initState_i];
            finalStates = [finalStates; finalState_ij];
            direction = [direction; dir];
        end
        if headings(i) > 0 && headings(i) < pi/4
            for j = 1:delta
                k = i-j;
                if k <= 0
                    neg_idx = length(headings) + k;
                else
                    neg_idx = k;
                end
                finalState2_ij = [0,0,neg_idx,0,0];
                initStates = [initStates; initState_i];
                finalStates = [finalStates; finalState2_ij];
                direction = [direction; dir];
            end
        end
    end
     
    
    
end
bvp_headings.initStates = initStates;
bvp_headings.finalStates = finalStates;
bvp_headings.direction = direction;
bvp_headings.heading_vec = headings;
end

