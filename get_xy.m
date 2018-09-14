function xy = get_xy(heading, disc, dir)
%GET_XY Summary of this function goes here
%   Detailed explanation goes here
switch heading 
    case 0 
        xy = dir*disc*[1,0];
    case atan(1/2)
        xy = dir*disc*[2,1];
    case pi/4 
        xy = dir*disc*[1,1];
    otherwise
        error("Current heading not specified, update get_xy if you want to use this")
end
        
end

