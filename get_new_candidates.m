function [new_x, new_y] = get_new_candidates(x_min, x_max, y_min, y_max, disc)
%If no feasible solution exists from previous candidates, continue with
%more candidates
%   
if y_min == y_max % Parallel movement
    new_x = [x_min-disc, x_max+disc];
    new_y = [y_min, y_min];
else
    new_y_min = y_min-disc;
    new_x_min = x_min-disc;
    new_x_max = x_max+disc;
    new_y_max = y_max+disc;
    y_vec_p = new_y_min:disc:new_y_max;
    y_vec_n = new_y_max:-disc:new_y_min;
    x_cn = new_x_min*ones(1,length(y_vec_p));
    x_cp = new_x_max*ones(1,length(y_vec_p));
    
    x_vec_n = x_max:-disc:x_min;
    x_vec_p = x_min:disc:x_max;
    y_cn = new_y_min*ones(1,length(x_vec_n));
    y_cp = new_y_max*ones(1,length(x_vec_n));
    
    new_x = [x_cp, x_vec_n, x_cn, x_vec_p];
    new_y = [y_vec_n, y_cn, y_vec_p, y_cp];
end
end

