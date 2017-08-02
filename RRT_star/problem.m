classdef problem
    properties
    dim  % dimension of problem 
    range % workspace /size: D x 2 
    map
    end
    
    methods
        function obj=problem(dim,range,sim_map)
        obj.dim=dim;
        obj.range=range;
        obj.map=sim_map;
        end
        
        
        
    
    end
    
end