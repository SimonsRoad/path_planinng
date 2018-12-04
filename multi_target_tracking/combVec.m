function d=combVec(v1,v2)
    [A,B] = meshgrid(v1,v2);
    c=cat(2,A',B');
    d=reshape(c,[],2);
end