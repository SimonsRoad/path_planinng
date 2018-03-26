classdef gridmap<handle
    properties
        origin
        dx
        dy
        Nx
        Ny
        map
    end
      
    methods
        function obj=gridmap(origin,dx,dy,Nx,Ny)
            obj.origin=origin;
            obj.dx=dx;
            obj.dy=dy;
            obj.Nx=Nx;
            obj.Ny=Ny;
            obj.map=ones(Nx,Ny)*0;
        end
        
        function xy=cell2xy(gridmap,i,j)
            x=gridmap.origin(1)+gridmap.dx*(i-1);
            y=gridmap.origin(2)+gridmap.dy*(j-1);   
            xy=[x,y];
        end
        
        function ij=xy2cell(gridmap,x,y)
            i=floor((x-gridmap.origin(1))/gridmap.dx)+1;
            j=floor((y-gridmap.origin(2))/gridmap.dy)+1;
            ij=[i,j];
        end
            
        function mapplot(gridmap)
            colormap(jet)
            
            imagesc(((gridmap.get_map')),'XData',[gridmap.origin(1) gridmap.origin(1)+gridmap.dx*...
                gridmap.Nx],'YData',[ gridmap.origin(2) gridmap.origin(2)+gridmap.dy*gridmap.Ny ])     
            xlabel('x')
            ylabel('y')
        end
        
        function obstacle_assign(gridmap,obs_origin,obs_scale,value)
            % obs_origin=[x,y]
            % obs_scale=[lx,ly]
            % assign_value?
            
            
            [lower_left_idx]=gridmap.xy2cell(obs_origin(1)-obs_scale(1),obs_origin(2)-obs_scale(2));
            [upper_right_idx]=gridmap.xy2cell(obs_origin(1)+obs_scale(1),obs_origin(2)+obs_scale(2));
            
            gridmap.map(lower_left_idx(1):upper_right_idx(1), lower_left_idx(2):upper_right_idx(2))=value;
              
        end
        
        function [p,samp_xy,samp_idx]=sample(gridmap)

            cut_prob=0.9; % if sampled occupancy is greater than this, it will be rejected  
            while 1
                
                samp_x=randi(gridmap.Nx);
                samp_y=randi(gridmap.Ny);
                p=gridmap.map(samp_x,samp_y);
                if p<cut_prob
                    break
                end
            end
            
            samp_xy=gridmap.cell2xy(samp_x,samp_y);
            samp_idx=[samp_x,samp_y];
                                      
        end
                   
        function map=get_map(gridmap)
            map=gridmap.map;
        end
        
        function [isobs,p]=colision(gridmap,x)
            cut_prob=0.9;
            ij=gridmap.xy2cell(x(1),x(2));
            isobs=false;
            p=gridmap.map(ij(1),ij(2));
            if p>cut_prob
                isobs=true;
            end
            
            
        end
        
        
    end

end
    