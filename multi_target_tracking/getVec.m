function v=getVec(azim,elev)
    % this function calculates the cosine vector of azim , elev
    v = [cos(elev)*cos(azim) cos(elev)*sin(azim) sin(elev)];
    
end