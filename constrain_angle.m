function angle_out = constrain_angle(angle)
%CONSTRAIN_ANGLE Returns angle between [0, 2*pi]

n = length(angle);
for j = 1:n
    if(angle(j) < 0 )
        angle(j) = angle(j) + 2*pi;
    elseif(angle(j) >= 2*pi)
        angle(j) = angle(j) - 2*pi;
    end
end
angle_out = angle;
end

