function  store_primitives(prims, path, iStart)

n = length(prims);
it = iStart;
for i = 1:n
   prims(i).store_primitive(path, it);
   it = it+1;
end

end

