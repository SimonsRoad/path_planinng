classdef obstacle2
    properties
        T % SE2 object from peter: 
        dim % length in x y  eg: [1 2]
    end
    
    methods
        function obj=obstacle2(T,dim)
            obj.T=T; 
            obj.dim=dim;
        end
        
        function plot(obstacle2)
            % for points of rect before applying T
            pnts=[obstacle2.dim(1) obstacle2.dim(2);-obstacle2.dim(1) obstacle2.dim(2);-obstacle2.dim(1) -obstacle2.dim(2);...
                obstacle2.dim(1) -obstacle2.dim(2)]; 
            
            pnts=pnts';
            % location of pnts after applying T
            pnts=obstacle2.T*pnts;
            
            % plotting using patch
            h=patch(pnts(1,:),pnts(2,:),'k','FaceAlpha',0.3);
            set(h,'edgecolor','k')
                        
        end
        
        function res=isobs(obstacle2,p_query)
            % p_query = 2 x N_query 
            [~,N]=size(p_query); % query could be N points 
            p_query=double(obstacle2.T)\[p_query(1,:);p_query(2,:);ones(1,N)];
            p_query=p_query(1:2,:);
            res=iswithin(p_query,[-obstacle2.dim(1) obstacle2.dim(1); -obstacle2.dim(2) obstacle2.dim(2)]);
         end
        
        
    end
    
    
end