function [x_cand, y_cand] = get_L_inf_candidates(r, dir)

        x_nn = 0:-1:-r;
        x_np = -r:1:0;
        x_pn = r:-1:1;
        x_pp = 1:1:r;
        y_p = -(r-1):1:(r-1);
        y_n = (r-1):-1:-(r-1);
        
        y_c1 = r*ones(1,length(x_nn));
        y_c2 = r*ones(1,length(x_pp));
        x_c = r*ones(1,length(y_p));
        if dir == 1
            x_cand = [x_np, x_pp, x_c, x_pn, x_nn, -x_c];
            y_cand = [y_c1, y_c2, y_n, -y_c2, -y_c1, y_p];
        else
            x_cand = [x_pn, x_nn, -x_c, x_np, x_pp, x_c];
            y_cand = [-y_c1, -y_c2, y_p, y_c1, y_c2, y_n];    
        end
        
end