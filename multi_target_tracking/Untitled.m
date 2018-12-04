try 
    vertices = lcon2vert([A_bound;A_intsec ],[;b_bound ;b_intsec]); % the vertices of this region   (this can be reducded to 2-dim , which we don't want)    
catch
    disp("shit")
end
