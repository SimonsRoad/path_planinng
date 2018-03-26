classdef obstacle
    properties
        T % SE3 object from peter: 
        dim % length in x y z from origin of box 
    end
    
    methods
        function obj=obstacle(T,dim)
            obj.T=T; 
            obj.dim=dim;
        end
        
        function plot(obstacle)
            % for points of rect before applying T
            p1=-obstacle.dim; p2=obstacle.dim;
            x=([0 1 1 0 0 0;1 1 0 0 1 1;1 1 0 0 1 1;0 1 1 0 0 0])*(p2(1)-p1(1))+p1(1);
            y=([0 0 1 1 0 0;0 1 1 0 0 0;0 1 1 0 1 1;0 0 1 1 1 1])*(p2(2)-p1(2))+p1(2);
            z=([0 0 0 0 0 1;0 0 0 0 0 1;1 1 1 1 0 1;1 1 1 1 0 1])*(p2(3)-p1(3))+p1(3);
            x=reshape(x,1,[]);
            y=reshape(y,1,[]);
            z=reshape(z,1,[]);
            
            pnts=[x;y;z];
            % location of pnts after applying T
            pnts=obstacle.T*pnts;
            
            x=reshape(pnts(1,:),4,6);            
            y=reshape(pnts(2,:),4,6);
            z=reshape(pnts(3,:),4,6);
            % plotting using patch
           for i=1:6
                h=patch(x(:,i),y(:,i),z(:,i),'k','FaceAlpha',0.2);
                set(h,'edgecolor','w')
           end
        end
        
        function res=isobs(obstacle,p_query)
            [~,N]=size(p_query); % query could be N points 
            p_query=double(obstacle.T)\[p_query(1,:);p_query(2,:);p_query(3,:);ones(1,N)];
            p_query=p_query(1:3,:);
            res=iswithin(p_query,[-obstacle.dim(1) obstacle.dim(1); -obstacle.dim(2) obstacle.dim(2);-obstacle.dim(3)  obstacle.dim(3)]);
         end
        
        
    end
    
    
end