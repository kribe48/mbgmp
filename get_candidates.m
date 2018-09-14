function [x_cand, y_cand] = get_candidates(x_res,y_res,disc, primType, finalHeading)
x_cand = [];
y_cand = [];
fixed = ones(1,5);
switch primType
    case 'heading'
        fixed(1) = 0;
        fixed(2) = 0;
    case 'parallel'
        fixed(1) = 0;
        if(finalHeading ~= 0)
            fixed(2) = 0;
        end
end

if fixed(1) == 0 && fixed(2) == 1
    x_dif = mod(x_res, disc); 
    if x_dif == 0
        %do nothing, already at a disc point
    else 
        x_cand = [x_res-x_dif,x_res+(disc-x_dif)];
        y_cand = [y_res, y_res];
    end
elseif fixed(1) == 0 && fixed(2) == 0
    x_dif = mod(x_res, disc); 
    y_dif = mod(y_res, disc);
    if x_dif == 0 && y_dif == 0
        %do nothing, already at disc point
    elseif x_dif == 0
        x_cand = [x_res, x_res, x_res-disc, x_res-disc, x_res+disc, x_res+disc];
        y_cand = [y_res - y_dif, y_res + (disc-y_dif), y_res - y_dif, y_res + (disc-y_dif), y_res - y_dif, y_res + (disc-y_dif)];
    elseif y_dif == 0
        y_cand = [y_res, y_res, y_res-disc, y_res-disc, y_res+disc, y_res+disc];
        x_cand = [x_res - x_dif, x_res + (disc-x_dif), x_res - x_dif, x_res + (disc-x_dif), x_res - x_dif, x_res + (disc-x_dif)];
    else
        x_cand = [x_res - x_dif, x_res - x_dif, x_res + (disc-x_dif), x_res + (disc-x_dif)];
        y_cand = [y_res - y_dif, y_res + (disc-y_dif), y_res - y_dif, y_res + (disc-y_dif)];
    end
end
    
end

