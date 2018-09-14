function mirrored_prims = mirror_primitives(prims)
%MIRROR_PRIMITIVES Summary of this function goes here
%   Detailed explanation goes here
n = length(prims);
mirrored_prims = [];
for i = 1:n
    mirrored_prims = [mirrored_prims, prims(i).mirror_primitive()];
end

