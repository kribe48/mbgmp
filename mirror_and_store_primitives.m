function mirror_and_store_primitives(primVec, filePath)
%MIRROR_AND_STORE_PRIMITIVES Summary of this function goes here
%   Detailed explanation goes here
itStart = 0;
primVec_mir = mirror_primitives(primVec);
store_primitives(primVec_mir,filePath,itStart);
end

